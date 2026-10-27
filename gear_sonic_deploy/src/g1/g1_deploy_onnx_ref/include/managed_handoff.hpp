#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>
#include <string_view>

#include "robot_parameters.hpp"


inline constexpr int kWalkrunFsmId = 801;
inline constexpr int kUserControlFsmId = 1000;
inline constexpr int kPassiveFsmId = 1;
inline constexpr int32_t kSwitchToUserControlApiId = 7110;
inline constexpr int32_t kSwitchToInternalControlApiId = 7111;

inline constexpr std::string_view kWalkrunReturnConfirmedEvent =
    "walkrun_return_confirmed";
inline constexpr std::string_view kEmergencyStopRequestedEvent =
    "emergency_stop_requested";
inline constexpr std::chrono::milliseconds kWalkrunReturnAckBroadcastDuration{
    500};
inline constexpr std::chrono::milliseconds kEmergencyStopBroadcastDuration{
    500};

struct HandoffStatus {
  std::string_view event;
  int fsm_id;
};

inline constexpr HandoffStatus MakeWalkrunReturnConfirmedStatus() {
  return {kWalkrunReturnConfirmedEvent, kWalkrunFsmId};
}

inline constexpr HandoffStatus MakeEmergencyStopRequestedStatus() {
  return {kEmergencyStopRequestedEvent, kPassiveFsmId};
}

inline bool IsWalkrunReturnAckBroadcastComplete(
    std::chrono::steady_clock::time_point confirmed_at,
    std::chrono::steady_clock::time_point now) {
  return now - confirmed_at >= kWalkrunReturnAckBroadcastDuration;
}

inline bool IsEmergencyStopBroadcastComplete(
    std::chrono::steady_clock::time_point requested_at,
    std::chrono::steady_clock::time_point now) {
  return now - requested_at >= kEmergencyStopBroadcastDuration;
}

inline constexpr std::string_view BodyCommandTopic(bool managed_handoff) {
  return managed_handoff ? "rt/user_lowcmd" : "rt/lowcmd";
}

inline constexpr bool ShouldEnableManagedHandoff(
    std::string_view input_type, bool disable_crc_check) {
  return input_type == "zmq_manager" && !disable_crc_check;
}

inline constexpr bool ShouldBypassDdsDestructors(
    bool managed_handoff, bool walkrun_return_confirmed) {
  return managed_handoff && walkrun_return_confirmed;
}

enum class ManagedInternalFsmMode {
  PASSIVE = 1,
  WALKRUN = 2,
};

class ManagedLocoClient {
 public:
  ManagedLocoClient();
  ~ManagedLocoClient();

  void SetTimeout(float timeout_seconds);
  void Init();
  int32_t GetFsmId(int& fsm_id);
  int32_t SwitchToUserCtrl();
  int32_t SwitchToInternalCtrl(ManagedInternalFsmMode mode);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};


enum class UserCtrlHandoffState {
  DISABLED,
  STABILIZING,
  READY,
  ENTRY_HOLD,
  REQUESTING_USER_CTRL,
  WAITING_USER_CTRL,
  VERIFYING_USER_SWITCH_FAILURE,
  BLENDING,
  ACTIVE,
  RETURN_STABILIZING,
  RETURN_READY,
  RETURN_HOLD,
  REQUESTING_WALKRUN,
  WAITING_WALKRUN,
  USER_OWNED_HOLD,
  EMERGENCY_DAMPING,
  WAITING_PASSIVE,
  COMPLETE,
};

enum class UserCtrlRejectReason {
  NONE,
  STALE_SAMPLE,
  NON_FINITE_VALUE,
  NOT_WALKRUN,
  NOT_USER_CONTROL,
  JOINT_MOVING,
  TORSO_TILTED,
  TORSO_MOVING,
  LOWER_BODY_POSE_MISMATCH,
  UPPER_BODY_POSE_MISMATCH,
  LOWER_BODY_ASYMMETRY,
};

struct UserCtrlObservation {
  std::chrono::steady_clock::time_point sample_time{};
  int fsm_id = -1;
  std::array<float, G1_NUM_MOTOR> joint_position{};
  std::array<float, G1_NUM_MOTOR> joint_velocity{};
  std::array<float, 3> torso_rpy{};
  std::array<float, 3> torso_angular_velocity{};
};

struct UserCtrlReturnReadiness {
  UserCtrlRejectReason reason = UserCtrlRejectReason::NONE;
  int max_lower_body_joint = -1;
  float max_lower_body_delta = 0.0F;
  float lower_body_delta_limit = 0.0F;
  int max_upper_body_joint = -1;
  float max_upper_body_delta = 0.0F;
  float upper_body_delta_limit = 0.0F;
  float max_joint_velocity = 0.0F;
  float torso_angular_speed = 0.0F;
  std::array<float, RightAnkleRoll + 1> reference_lower_body{};
  std::array<float, RightAnkleRoll + 1> current_lower_body{};
  std::array<float, G1_NUM_MOTOR> reference_upper_body{};
  std::array<float, G1_NUM_MOTOR> current_upper_body{};
  std::array<float, 3> torso_rpy{};
};

class UserCtrlHandoffController {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit UserCtrlHandoffController(bool enabled);

  void Observe(const UserCtrlObservation& observation, TimePoint now);
  std::optional<std::uint64_t> RequestEntry(TimePoint now);
  bool AcknowledgeHoldWrite(std::uint64_t generation, TimePoint now);
  void MarkUserSwitchAccepted(TimePoint now);
  void MarkUserSwitchRpcFailed(TimePoint now);
  void MarkPolicyBlendComplete();
  std::array<float, G1_NUM_MOTOR> TransformPolicyPosition(
      const std::array<float, G1_NUM_MOTOR>& policy_position,
      TimePoint now);

  bool RequestReturnReferenceCapture(TimePoint now);
  bool CancelReturnReferenceCapture();
  bool has_return_reference() const;
  const std::optional<std::array<float, G1_NUM_MOTOR>>&
  return_reference_position() const;

  bool RequestReturn();
  bool RequestStop(bool return_ready, TimePoint now);
  std::optional<std::uint64_t> BeginReturn(TimePoint now);
  void MarkInternalSwitchAccepted(TimePoint now);
  void MarkInternalSwitchRejected();
  bool RetryInternalSwitch(TimePoint now);
  std::optional<UserCtrlReturnReadiness> return_readiness(
      TimePoint now) const;

  void RequestEmergencyStop(TimePoint now);
  void MarkPassiveSwitchAccepted(TimePoint now);
  void CheckTransitionTimeout(TimePoint now);

  UserCtrlHandoffState state() const;
  UserCtrlRejectReason rejection_reason() const;
  const std::array<float, G1_NUM_MOTOR>& hold_position() const;
  bool body_publication_enabled() const;
  bool policy_enabled() const;
  bool ready_to_request_user_ctrl() const;
  bool ready_to_request_walkrun() const;
  bool ready_to_request_passive() const;

 private:
  UserCtrlRejectReason ValidateStableObservation(
      const UserCtrlObservation& observation, TimePoint now,
      int required_fsm, bool require_return_reference = true) const;
  UserCtrlRejectReason ValidateLowerBodyGeometry(
      const UserCtrlObservation& observation) const;
  void UpdateReturnReferenceCapture(
      const UserCtrlObservation& observation, TimePoint now);
  void ObserveStability(const UserCtrlObservation& observation, TimePoint now,
                        int required_fsm, UserCtrlHandoffState ready_state);
  void CaptureHold();

  bool enabled_ = false;
  UserCtrlHandoffState state_ = UserCtrlHandoffState::DISABLED;
  UserCtrlRejectReason rejection_reason_ = UserCtrlRejectReason::NONE;
  std::optional<TimePoint> stable_since_;
  std::optional<TimePoint> return_stabilization_started_at_;
  std::optional<TimePoint> transition_deadline_;
  std::optional<TimePoint> blend_started_at_;
  std::optional<UserCtrlObservation> latest_observation_;
  std::optional<std::array<float, G1_NUM_MOTOR>> return_reference_position_;
  std::optional<TimePoint> return_reference_stable_since_;
  std::vector<std::array<float, G1_NUM_MOTOR>>
      return_reference_samples_;
  bool return_reference_capture_attempted_ = false;
  bool return_reference_capture_pending_ = false;
  std::array<float, G1_NUM_MOTOR> hold_position_{};
  std::array<float, G1_NUM_MOTOR> policy_offset_{};
  std::uint64_t hold_generation_ = 0;
  std::optional<std::uint64_t> expected_hold_generation_;
};

const char* ToString(UserCtrlHandoffState state);
const char* ToString(UserCtrlRejectReason reason);
