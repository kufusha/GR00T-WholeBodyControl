#include "managed_handoff.hpp"

#include <algorithm>
#include <cmath>

#if defined(__linux__)
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#endif

namespace {

constexpr auto kMaximumSampleAge = std::chrono::milliseconds(100);
constexpr auto kStabilityWindow = std::chrono::seconds(1);
constexpr auto kReturnIdleSettlementWindow = std::chrono::seconds(1);
constexpr auto kTransitionTimeout = std::chrono::seconds(3);
constexpr auto kPolicyBlendDuration = std::chrono::seconds(2);
constexpr auto kReturnReferenceStabilityWindow =
    std::chrono::milliseconds(300);
constexpr float kMaximumJointVelocity = 0.2F;
constexpr float kMaximumTiltRadians = 10.0F * 3.14159265358979323846F / 180.0F;
constexpr float kMaximumTorsoAngularSpeed = 0.25F;
constexpr float kMaximumLowerBodyReferenceDelta = 0.45F;
constexpr float kMaximumUpperBodyReferenceDelta = 0.45F;
constexpr float kMaximumHipYawMirrorError = 0.15F;
constexpr float kMaximumKneeDifference = 0.12F;
constexpr float kMaximumKneeFlexion = 0.80F;
constexpr float kMaximumAnklePitch = 0.50F;

bool IsFinite(float value) {
  return std::isfinite(value);
}

bool IsImmediateReturnRejection(UserCtrlRejectReason reason) {
  return reason != UserCtrlRejectReason::NONE &&
         reason != UserCtrlRejectReason::JOINT_MOVING &&
         reason != UserCtrlRejectReason::TORSO_MOVING &&
         reason != UserCtrlRejectReason::LOWER_BODY_POSE_MISMATCH &&
         reason != UserCtrlRejectReason::UPPER_BODY_POSE_MISMATCH;
}

}  // namespace

UserCtrlHandoffController::UserCtrlHandoffController(bool enabled)
    : enabled_(enabled),
      state_(enabled ? UserCtrlHandoffState::STABILIZING
                     : UserCtrlHandoffState::DISABLED) {}

UserCtrlRejectReason UserCtrlHandoffController::ValidateStableObservation(
    const UserCtrlObservation& observation, TimePoint now,
    int required_fsm, bool require_return_reference) const {
  if (observation.sample_time > now ||
      now - observation.sample_time > kMaximumSampleAge) {
    return UserCtrlRejectReason::STALE_SAMPLE;
  }
  if (required_fsm == kWalkrunFsmId && observation.fsm_id != kWalkrunFsmId) {
    return UserCtrlRejectReason::NOT_WALKRUN;
  }
  if (required_fsm == kUserControlFsmId &&
      observation.fsm_id != kUserControlFsmId) {
    return UserCtrlRejectReason::NOT_USER_CONTROL;
  }
  for (float value : observation.joint_position) {
    if (!IsFinite(value)) { return UserCtrlRejectReason::NON_FINITE_VALUE; }
  }
  if (required_fsm == kUserControlFsmId && require_return_reference) {
    if (!return_reference_position_) {
      return UserCtrlRejectReason::LOWER_BODY_POSE_MISMATCH;
    }
    for (int joint = LeftHipPitch; joint <= RightAnkleRoll; ++joint) {
      if (std::fabs(observation.joint_position[joint] -
                    (*return_reference_position_)[joint]) >
          kMaximumLowerBodyReferenceDelta) {
        return UserCtrlRejectReason::LOWER_BODY_POSE_MISMATCH;
      }
    }
    for (int joint = WaistYaw; joint < G1_NUM_MOTOR; ++joint) {
      if (std::fabs(observation.joint_position[joint] -
                    (*return_reference_position_)[joint]) >
          kMaximumUpperBodyReferenceDelta) {
        return UserCtrlRejectReason::UPPER_BODY_POSE_MISMATCH;
      }
    }
  }
  for (float value : observation.joint_velocity) {
    if (!IsFinite(value)) { return UserCtrlRejectReason::NON_FINITE_VALUE; }
    if (std::fabs(value) > kMaximumJointVelocity) {
      return UserCtrlRejectReason::JOINT_MOVING;
    }
  }
  for (float value : observation.torso_rpy) {
    if (!IsFinite(value)) { return UserCtrlRejectReason::NON_FINITE_VALUE; }
  }
  for (float value : observation.torso_angular_velocity) {
    if (!IsFinite(value)) { return UserCtrlRejectReason::NON_FINITE_VALUE; }
  }
  if (std::fabs(observation.torso_rpy[0]) > kMaximumTiltRadians ||
      std::fabs(observation.torso_rpy[1]) > kMaximumTiltRadians) {
    return UserCtrlRejectReason::TORSO_TILTED;
  }
  const float angular_speed = std::sqrt(
      observation.torso_angular_velocity[0] *
          observation.torso_angular_velocity[0] +
      observation.torso_angular_velocity[1] *
          observation.torso_angular_velocity[1] +
      observation.torso_angular_velocity[2] *
          observation.torso_angular_velocity[2]);
  if (angular_speed > kMaximumTorsoAngularSpeed) {
    return UserCtrlRejectReason::TORSO_MOVING;
  }
  if (required_fsm == kUserControlFsmId) {
    return ValidateLowerBodyGeometry(observation);
  }
  return UserCtrlRejectReason::NONE;
}

UserCtrlRejectReason UserCtrlHandoffController::ValidateLowerBodyGeometry(
    const UserCtrlObservation& observation) const {
  const auto& q = observation.joint_position;
  if (std::fabs(q[LeftHipYaw] + q[RightHipYaw]) >
          kMaximumHipYawMirrorError ||
      std::fabs(q[LeftKnee] - q[RightKnee]) >
          kMaximumKneeDifference ||
      std::fabs(q[LeftKnee]) > kMaximumKneeFlexion ||
      std::fabs(q[RightKnee]) > kMaximumKneeFlexion ||
      std::fabs(q[LeftAnklePitch]) > kMaximumAnklePitch ||
      std::fabs(q[RightAnklePitch]) > kMaximumAnklePitch) {
    return UserCtrlRejectReason::LOWER_BODY_ASYMMETRY;
  }
  return UserCtrlRejectReason::NONE;
}

void UserCtrlHandoffController::UpdateReturnReferenceCapture(
    const UserCtrlObservation& observation, TimePoint now) {
  if (!return_reference_capture_pending_) { return; }

  const auto reason = ValidateStableObservation(
      observation, now, kUserControlFsmId, false);
  if (reason != UserCtrlRejectReason::NONE) {
    return_reference_stable_since_.reset();
    return_reference_samples_.clear();
  } else {
    if (!return_reference_stable_since_) {
      return_reference_stable_since_ = now;
    }
    return_reference_samples_.push_back(observation.joint_position);
    if (now - *return_reference_stable_since_ >=
        kReturnReferenceStabilityWindow) {
      std::array<float, G1_NUM_MOTOR> median{};
      for (int joint = 0; joint < G1_NUM_MOTOR; ++joint) {
        std::vector<float> values;
        values.reserve(return_reference_samples_.size());
        for (const auto& sample : return_reference_samples_) {
          values.push_back(sample[joint]);
        }
        const auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        median[joint] = *middle;
      }
      return_reference_position_ = median;
      return_reference_capture_pending_ = false;
      return_reference_stable_since_.reset();
      return_reference_samples_.clear();
      return;
    }
  }
}

void UserCtrlHandoffController::ObserveStability(
    const UserCtrlObservation& observation, TimePoint now, int required_fsm,
    UserCtrlHandoffState ready_state) {
  rejection_reason_ = ValidateStableObservation(observation, now, required_fsm);
  if (rejection_reason_ != UserCtrlRejectReason::NONE) {
    stable_since_.reset();
    return;
  }
  if (!stable_since_) { stable_since_ = now; }
  if (now - *stable_since_ >= kStabilityWindow) { state_ = ready_state; }
}

void UserCtrlHandoffController::Observe(
    const UserCtrlObservation& observation, TimePoint now) {
  if (!enabled_) { return; }
  latest_observation_ = observation;
  UpdateReturnReferenceCapture(observation, now);

  switch (state_) {
    case UserCtrlHandoffState::STABILIZING:
      ObserveStability(observation, now, kWalkrunFsmId,
                       UserCtrlHandoffState::READY);
      break;
    case UserCtrlHandoffState::WAITING_USER_CTRL:
      if (observation.fsm_id == kUserControlFsmId) {
        transition_deadline_.reset();
        state_ = UserCtrlHandoffState::BLENDING;
      }
      break;
    case UserCtrlHandoffState::VERIFYING_USER_SWITCH_FAILURE:
      if (observation.fsm_id == kUserControlFsmId) {
        transition_deadline_.reset();
        state_ = UserCtrlHandoffState::BLENDING;
      } else if (observation.fsm_id == kWalkrunFsmId) {
        transition_deadline_.reset();
        stable_since_.reset();
        state_ = UserCtrlHandoffState::STABILIZING;
      }
      break;
    case UserCtrlHandoffState::RETURN_STABILIZING:
      if (!return_stabilization_started_at_) {
        return_stabilization_started_at_ = now;
      }
      ObserveStability(observation, now, kUserControlFsmId,
                       UserCtrlHandoffState::RETURN_READY);
      if (state_ == UserCtrlHandoffState::RETURN_STABILIZING &&
          rejection_reason_ != UserCtrlRejectReason::NONE &&
          (IsImmediateReturnRejection(rejection_reason_) ||
           now - *return_stabilization_started_at_ >=
               kReturnIdleSettlementWindow)) {
        RequestEmergencyStop(now);
      }
      break;
    case UserCtrlHandoffState::WAITING_WALKRUN:
      if (observation.fsm_id == kWalkrunFsmId) {
        transition_deadline_.reset();
        state_ = UserCtrlHandoffState::COMPLETE;
      }
      break;
    case UserCtrlHandoffState::WAITING_PASSIVE:
      if (observation.fsm_id == kPassiveFsmId) {
        transition_deadline_.reset();
        state_ = UserCtrlHandoffState::COMPLETE;
      }
      break;
    default:
      break;
  }
}

void UserCtrlHandoffController::CaptureHold() {
  if (!latest_observation_) { return; }
  hold_position_ = latest_observation_->joint_position;
  ++hold_generation_;
  expected_hold_generation_ = hold_generation_;
}

std::optional<std::uint64_t> UserCtrlHandoffController::RequestEntry(
    TimePoint) {
  if (state_ != UserCtrlHandoffState::READY || !latest_observation_) {
    return std::nullopt;
  }
  CaptureHold();
  state_ = UserCtrlHandoffState::ENTRY_HOLD;
  return expected_hold_generation_;
}

bool UserCtrlHandoffController::AcknowledgeHoldWrite(
    std::uint64_t generation, TimePoint) {
  if (!expected_hold_generation_ || generation != *expected_hold_generation_) {
    return false;
  }
  expected_hold_generation_.reset();
  if (state_ == UserCtrlHandoffState::ENTRY_HOLD) {
    state_ = UserCtrlHandoffState::REQUESTING_USER_CTRL;
    return true;
  }
  if (state_ == UserCtrlHandoffState::RETURN_HOLD) {
    state_ = UserCtrlHandoffState::REQUESTING_WALKRUN;
    return true;
  }
  return false;
}

void UserCtrlHandoffController::MarkUserSwitchAccepted(TimePoint now) {
  if (state_ != UserCtrlHandoffState::REQUESTING_USER_CTRL) { return; }
  state_ = UserCtrlHandoffState::WAITING_USER_CTRL;
  transition_deadline_ = now + kTransitionTimeout;
}

bool UserCtrlHandoffController::RequestReturnReferenceCapture(TimePoint) {
  if ((state_ != UserCtrlHandoffState::BLENDING &&
       state_ != UserCtrlHandoffState::ACTIVE) ||
      return_reference_capture_attempted_) {
    return false;
  }
  return_reference_capture_attempted_ = true;
  return_reference_capture_pending_ = true;
  return_reference_stable_since_.reset();
  return_reference_samples_.clear();
  return true;
}

bool UserCtrlHandoffController::CancelReturnReferenceCapture() {
  if (!return_reference_capture_pending_ || return_reference_position_) {
    return false;
  }
  return_reference_capture_attempted_ = false;
  return_reference_capture_pending_ = false;
  return_reference_stable_since_.reset();
  return_reference_samples_.clear();
  return true;
}

bool UserCtrlHandoffController::has_return_reference() const {
  return return_reference_position_.has_value();
}

const std::optional<std::array<float, G1_NUM_MOTOR>>&
UserCtrlHandoffController::return_reference_position() const {
  return return_reference_position_;
}

void UserCtrlHandoffController::MarkUserSwitchRpcFailed(TimePoint now) {
  if (state_ != UserCtrlHandoffState::REQUESTING_USER_CTRL) { return; }
  state_ = UserCtrlHandoffState::VERIFYING_USER_SWITCH_FAILURE;
  transition_deadline_ = now + kTransitionTimeout;
}

std::optional<UserCtrlReturnReadiness>
UserCtrlHandoffController::return_readiness(TimePoint now) const {
  if (!latest_observation_ || !return_reference_position_) {
    return std::nullopt;
  }

  UserCtrlReturnReadiness readiness;
  readiness.reason =
      ValidateStableObservation(*latest_observation_, now, kUserControlFsmId);
  readiness.lower_body_delta_limit = kMaximumLowerBodyReferenceDelta;
  readiness.upper_body_delta_limit = kMaximumUpperBodyReferenceDelta;
  readiness.torso_rpy = latest_observation_->torso_rpy;

  for (int joint = LeftHipPitch; joint <= RightAnkleRoll; ++joint) {
    const float entry = (*return_reference_position_)[joint];
    const float current = latest_observation_->joint_position[joint];
    const float delta = std::fabs(current - entry);
    readiness.reference_lower_body[joint] = entry;
    readiness.current_lower_body[joint] = current;
    if (delta > readiness.max_lower_body_delta) {
      readiness.max_lower_body_delta = delta;
      readiness.max_lower_body_joint = joint;
    }
  }

  for (int joint = WaistYaw; joint < G1_NUM_MOTOR; ++joint) {
    const float entry = (*return_reference_position_)[joint];
    const float current = latest_observation_->joint_position[joint];
    const float delta = std::fabs(current - entry);
    readiness.reference_upper_body[joint] = entry;
    readiness.current_upper_body[joint] = current;
    if (delta > readiness.max_upper_body_delta) {
      readiness.max_upper_body_delta = delta;
      readiness.max_upper_body_joint = joint;
    }
  }

  for (float velocity : latest_observation_->joint_velocity) {
    readiness.max_joint_velocity =
        std::max(readiness.max_joint_velocity, std::fabs(velocity));
  }

  const auto& angular_velocity =
      latest_observation_->torso_angular_velocity;
  readiness.torso_angular_speed = std::sqrt(
      angular_velocity[0] * angular_velocity[0] +
      angular_velocity[1] * angular_velocity[1] +
      angular_velocity[2] * angular_velocity[2]);
  return readiness;
}

void UserCtrlHandoffController::MarkPolicyBlendComplete() {
  if (state_ == UserCtrlHandoffState::BLENDING) {
    state_ = UserCtrlHandoffState::ACTIVE;
    blend_started_at_.reset();
  }
}

std::array<float, G1_NUM_MOTOR>
UserCtrlHandoffController::TransformPolicyPosition(
    const std::array<float, G1_NUM_MOTOR>& policy_position,
    TimePoint now) {
  if (state_ != UserCtrlHandoffState::BLENDING) { return policy_position; }
  if (!blend_started_at_) {
    blend_started_at_ = now;
    for (std::size_t i = 0; i < policy_offset_.size(); ++i) {
      policy_offset_[i] = hold_position_[i] - policy_position[i];
    }
  }

  const float alpha = std::clamp(
      std::chrono::duration<float>(now - *blend_started_at_).count() /
          std::chrono::duration<float>(kPolicyBlendDuration).count(),
      0.0F, 1.0F);
  std::array<float, G1_NUM_MOTOR> transformed = policy_position;
  for (std::size_t i = 0; i < transformed.size(); ++i) {
    transformed[i] += policy_offset_[i] * (1.0F - alpha);
  }
  if (alpha >= 1.0F) {
    state_ = UserCtrlHandoffState::ACTIVE;
    blend_started_at_.reset();
  }
  return transformed;
}

bool UserCtrlHandoffController::RequestReturn() {
  if (state_ != UserCtrlHandoffState::BLENDING &&
      state_ != UserCtrlHandoffState::ACTIVE) {
    return false;
  }
  state_ = UserCtrlHandoffState::RETURN_STABILIZING;
  stable_since_.reset();
  return_stabilization_started_at_.reset();
  rejection_reason_ = UserCtrlRejectReason::NONE;
  return true;
}

bool UserCtrlHandoffController::RequestStop(
    bool return_ready, TimePoint now) {
  if (state_ != UserCtrlHandoffState::BLENDING &&
      state_ != UserCtrlHandoffState::ACTIVE) {
    return false;
  }
  if (!return_ready) {
    RequestEmergencyStop(now);
    return true;
  }
  return RequestReturn();
}

std::optional<std::uint64_t> UserCtrlHandoffController::BeginReturn(
    TimePoint) {
  if (state_ != UserCtrlHandoffState::RETURN_READY || !latest_observation_) {
    return std::nullopt;
  }
  CaptureHold();
  state_ = UserCtrlHandoffState::RETURN_HOLD;
  return expected_hold_generation_;
}

void UserCtrlHandoffController::MarkInternalSwitchAccepted(TimePoint now) {
  if (state_ != UserCtrlHandoffState::REQUESTING_WALKRUN) { return; }
  state_ = UserCtrlHandoffState::WAITING_WALKRUN;
  transition_deadline_ = now + kTransitionTimeout;
}

void UserCtrlHandoffController::MarkInternalSwitchRejected() {
  if (state_ == UserCtrlHandoffState::REQUESTING_WALKRUN) {
    state_ = UserCtrlHandoffState::USER_OWNED_HOLD;
  }
}

bool UserCtrlHandoffController::RetryInternalSwitch(TimePoint) {
  if (state_ != UserCtrlHandoffState::USER_OWNED_HOLD) { return false; }
  state_ = UserCtrlHandoffState::REQUESTING_WALKRUN;
  transition_deadline_.reset();
  return true;
}

void UserCtrlHandoffController::RequestEmergencyStop(TimePoint) {
  if (!enabled_ || state_ == UserCtrlHandoffState::DISABLED ||
      state_ == UserCtrlHandoffState::COMPLETE) {
    return;
  }
  state_ = UserCtrlHandoffState::EMERGENCY_DAMPING;
  stable_since_.reset();
  return_stabilization_started_at_.reset();
  transition_deadline_.reset();
}

void UserCtrlHandoffController::MarkPassiveSwitchAccepted(TimePoint now) {
  if (state_ != UserCtrlHandoffState::EMERGENCY_DAMPING) { return; }
  state_ = UserCtrlHandoffState::WAITING_PASSIVE;
  transition_deadline_ = now + kTransitionTimeout;
}

void UserCtrlHandoffController::CheckTransitionTimeout(TimePoint now) {
  if (!transition_deadline_ || now <= *transition_deadline_) { return; }
  transition_deadline_.reset();
  if (state_ == UserCtrlHandoffState::WAITING_USER_CTRL ||
      state_ == UserCtrlHandoffState::VERIFYING_USER_SWITCH_FAILURE ||
      state_ == UserCtrlHandoffState::WAITING_WALKRUN ||
      state_ == UserCtrlHandoffState::WAITING_PASSIVE) {
    state_ = UserCtrlHandoffState::USER_OWNED_HOLD;
  }
}

UserCtrlHandoffState UserCtrlHandoffController::state() const {
  return state_;
}

UserCtrlRejectReason UserCtrlHandoffController::rejection_reason() const {
  return rejection_reason_;
}

const std::array<float, G1_NUM_MOTOR>&
UserCtrlHandoffController::hold_position() const {
  return hold_position_;
}

bool UserCtrlHandoffController::body_publication_enabled() const {
  switch (state_) {
    case UserCtrlHandoffState::ENTRY_HOLD:
    case UserCtrlHandoffState::REQUESTING_USER_CTRL:
    case UserCtrlHandoffState::WAITING_USER_CTRL:
    case UserCtrlHandoffState::VERIFYING_USER_SWITCH_FAILURE:
    case UserCtrlHandoffState::BLENDING:
    case UserCtrlHandoffState::ACTIVE:
    case UserCtrlHandoffState::RETURN_STABILIZING:
    case UserCtrlHandoffState::RETURN_READY:
    case UserCtrlHandoffState::RETURN_HOLD:
    case UserCtrlHandoffState::REQUESTING_WALKRUN:
    case UserCtrlHandoffState::WAITING_WALKRUN:
    case UserCtrlHandoffState::USER_OWNED_HOLD:
    case UserCtrlHandoffState::EMERGENCY_DAMPING:
    case UserCtrlHandoffState::WAITING_PASSIVE:
      return true;
    default:
      return false;
  }
}

bool UserCtrlHandoffController::policy_enabled() const {
  return state_ == UserCtrlHandoffState::BLENDING ||
         state_ == UserCtrlHandoffState::ACTIVE ||
         state_ == UserCtrlHandoffState::RETURN_STABILIZING ||
         state_ == UserCtrlHandoffState::RETURN_READY;
}

bool UserCtrlHandoffController::ready_to_request_user_ctrl() const {
  return state_ == UserCtrlHandoffState::REQUESTING_USER_CTRL;
}

bool UserCtrlHandoffController::ready_to_request_walkrun() const {
  return state_ == UserCtrlHandoffState::REQUESTING_WALKRUN;
}

bool UserCtrlHandoffController::ready_to_request_passive() const {
  return state_ == UserCtrlHandoffState::EMERGENCY_DAMPING;
}

const char* ToString(UserCtrlHandoffState state) {
  switch (state) {
    case UserCtrlHandoffState::DISABLED: return "DISABLED";
    case UserCtrlHandoffState::STABILIZING: return "STABILIZING";
    case UserCtrlHandoffState::READY: return "READY";
    case UserCtrlHandoffState::ENTRY_HOLD: return "ENTRY_HOLD";
    case UserCtrlHandoffState::REQUESTING_USER_CTRL: return "REQUESTING_USER_CTRL";
    case UserCtrlHandoffState::WAITING_USER_CTRL: return "WAITING_USER_CTRL";
    case UserCtrlHandoffState::VERIFYING_USER_SWITCH_FAILURE:
      return "VERIFYING_USER_SWITCH_FAILURE";
    case UserCtrlHandoffState::BLENDING: return "BLENDING";
    case UserCtrlHandoffState::ACTIVE: return "ACTIVE";
    case UserCtrlHandoffState::RETURN_STABILIZING: return "RETURN_STABILIZING";
    case UserCtrlHandoffState::RETURN_READY: return "RETURN_READY";
    case UserCtrlHandoffState::RETURN_HOLD: return "RETURN_HOLD";
    case UserCtrlHandoffState::REQUESTING_WALKRUN: return "REQUESTING_WALKRUN";
    case UserCtrlHandoffState::WAITING_WALKRUN: return "WAITING_WALKRUN";
    case UserCtrlHandoffState::USER_OWNED_HOLD: return "USER_OWNED_HOLD";
    case UserCtrlHandoffState::EMERGENCY_DAMPING: return "EMERGENCY_DAMPING";
    case UserCtrlHandoffState::WAITING_PASSIVE: return "WAITING_PASSIVE";
    case UserCtrlHandoffState::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

const char* ToString(UserCtrlRejectReason reason) {
  switch (reason) {
    case UserCtrlRejectReason::NONE: return "none";
    case UserCtrlRejectReason::STALE_SAMPLE: return "stale sample";
    case UserCtrlRejectReason::NON_FINITE_VALUE: return "non-finite value";
    case UserCtrlRejectReason::NOT_WALKRUN: return "not WALKRUN";
    case UserCtrlRejectReason::NOT_USER_CONTROL: return "not user control";
    case UserCtrlRejectReason::JOINT_MOVING: return "joint moving";
    case UserCtrlRejectReason::TORSO_TILTED: return "torso tilted";
    case UserCtrlRejectReason::TORSO_MOVING: return "torso moving";
    case UserCtrlRejectReason::LOWER_BODY_POSE_MISMATCH:
      return "lower-body pose differs from POSE reference";
    case UserCtrlRejectReason::UPPER_BODY_POSE_MISMATCH:
      return "upper-body pose differs from POSE reference";
    case UserCtrlRejectReason::LOWER_BODY_ASYMMETRY:
      return "lower-body pose is asymmetric or outside safe limits";
  }
  return "unknown";
}

#if defined(__linux__)
class ManagedLocoClient::Impl : public unitree::robot::g1::LocoClient {
 public:
  void InitManaged() {
    unitree::robot::g1::LocoClient::Init();
    RegistApi(kSwitchToUserControlApiId, 0);
    RegistApi(kSwitchToInternalControlApiId, 0);
  }

  int32_t SwitchToUserCtrlManaged() {
    unitree::robot::go2::JsonizeDataBool request;
    std::string response;
    return Call(kSwitchToUserControlApiId,
                unitree::common::ToJsonString(request), response);
  }

  int32_t SwitchToInternalCtrlManaged(ManagedInternalFsmMode mode) {
    unitree::robot::go2::JsonizeDataInt request;
    request.data = static_cast<int>(mode);
    std::string response;
    return Call(kSwitchToInternalControlApiId,
                unitree::common::ToJsonString(request), response);
  }
};
#else
class ManagedLocoClient::Impl {};
#endif

ManagedLocoClient::ManagedLocoClient() : impl_(std::make_unique<Impl>()) {}

ManagedLocoClient::~ManagedLocoClient() = default;

void ManagedLocoClient::SetTimeout(float timeout_seconds) {
#if defined(__linux__)
  impl_->SetTimeout(timeout_seconds);
#else
  (void)timeout_seconds;
#endif
}
void ManagedLocoClient::Init() {
#if defined(__linux__)
  impl_->InitManaged();
#endif
}
int32_t ManagedLocoClient::GetFsmId(int& fsm_id) {
#if defined(__linux__)
  return impl_->GetFsmId(fsm_id);
#else
  fsm_id = -1;
  return -1;
#endif
}

int32_t ManagedLocoClient::SwitchToUserCtrl() {
#if defined(__linux__)
  return impl_->SwitchToUserCtrlManaged();
#else
  return -1;
#endif
}

int32_t ManagedLocoClient::SwitchToInternalCtrl(
    ManagedInternalFsmMode mode) {
#if defined(__linux__)
  return impl_->SwitchToInternalCtrlManaged(mode);
#else
  (void)mode;
  return -1;
#endif
}
