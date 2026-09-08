#ifndef INSPIRE_HANDS_HPP
#define INSPIRE_HANDS_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

/**
 * @brief Adapter for controlling Inspire RH56DFX hands from Dex3-compatible
 *        7-DOF hand commands used by GR00T / GEAR-SONIC.
 *
 * GEAR-SONIC hand command order (7 DOF per hand):
 *   [thumb_0, thumb_1, thumb_2,
 *    index_0, index_1,
 *    middle_0, middle_1]
 *
 * Inspire hand order (6 DOF per hand):
 *   [pinky, ring, middle, index, thumb_bend, thumb_rotate]
 *
 * Inspire DDS layout:
 *   rt/inspire/cmd
 *     cmds()[0..5]   : right hand
 *     cmds()[6..11]  : left hand
 *
 *   rt/inspire/state
 *     states()[0..5]  : right hand
 *     states()[6..11] : left hand
 *
 * Inspire position convention:
 *   0.0 = closed
 *   1.0 = open
 */
class InspireHands
{
public:
    static constexpr int DEX3_DOF = 7;
    static constexpr int INSPIRE_DOF = 6;
    static constexpr int TOTAL_INSPIRE_DOF = 12;

    using Dex3Command = std::array<double, DEX3_DOF>;
    using InspireCommand = std::array<double, INSPIRE_DOF>;

    InspireHands() = default;

    /**
     * @brief Initialize DDS publisher/subscriber.
     *
     * If network_interface is empty, ChannelFactory initialization is skipped
     * because G1Deploy has already initialized it.
     */
    void initialize(const std::string& network_interface)
    {
        if (!network_interface.empty())
        {
            unitree::robot::ChannelFactory::Instance()->Init(
                0, network_interface.c_str());
        }

        publisher_ =
            std::make_shared<
                unitree::robot::ChannelPublisher<
                    unitree_go::msg::dds_::MotorCmds_>>(
                        "rt/inspire/cmd");

        subscriber_ =
            std::make_shared<
                unitree::robot::ChannelSubscriber<
                    unitree_go::msg::dds_::MotorStates_>>(
                        "rt/inspire/state");

        publisher_->InitChannel();

        subscriber_->InitChannel(
            [this](const void* message)
            {
                const auto* state =
                    static_cast<
                        const unitree_go::msg::dds_::MotorStates_*>(
                            message);

                if (state == nullptr ||
                    state->states().size() < TOTAL_INSPIRE_DOF)
                {
                    return;
                }

                std::lock_guard<std::mutex> lock(state_mutex_);

                for (int i = 0; i < TOTAL_INSPIRE_DOF; ++i)
                {
                    actual_inspire_state_[i] =
                        state->states()[i].q();
                }

                has_actual_state_ = true;
            },
            1);

        // GEAR-SONIC / Dex3 convention:
        // q = 0 corresponds to the open pose.
        left_dex3_command_.fill(0.0);
        right_dex3_command_.fill(0.0);

        // Inspire convention:
        // q = 1 corresponds to the open pose.
        dds_command_.cmds().resize(TOTAL_INSPIRE_DOF);
        for (int i = 0; i < TOTAL_INSPIRE_DOF; ++i)
        {
            dds_command_.cmds()[i].q() = 1.0;
        }
    }

    /**
     * @brief Set maximum closing ratio.
     *
     * 1.0 = full closure allowed
     * 0.2 = only 20% closure allowed
     */
    void SetMaxCloseRatio(double ratio)
    {
        max_close_ratio_.store(
            std::clamp(ratio, 0.2, 1.0),
            std::memory_order_relaxed);
    }

    double GetMaxCloseRatio() const
    {
        return max_close_ratio_.load(
            std::memory_order_relaxed);
    }

    /**
     * @brief Store a Dex3-compatible 7-DOF command.
     *
     * Conversion to Inspire's 6-DOF command is performed in writeOnce().
     */
    void setAllJointsCommand(
        bool is_left,
        const Dex3Command& q)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        if (is_left)
        {
            left_dex3_command_ = q;
        }
        else
        {
            right_dex3_command_ = q;
        }
    }

    /**
     * @brief Open one hand.
     *
     * Dex3 q=0 is the open pose.
     */
    void open(bool is_left)
    {
        Dex3Command q{};
        q.fill(0.0);
        setAllJointsCommand(is_left, q);
    }

    /**
     * @brief Move one hand to approximately half-closed pose.
     *
     * These values correspond to the midpoint poses used by Dex3Hands::close().
     */
    void close(bool is_left)
    {
        static constexpr Dex3Command LEFT_CLOSE = {
            0.0,
            0.163,
            0.875,
            -0.785,
            -0.875,
            -0.785,
            -0.875
        };

        static constexpr Dex3Command RIGHT_CLOSE = {
            0.0,
            -0.154,
            -0.875,
            0.785,
            0.875,
            0.785,
            0.875
        };

        setAllJointsCommand(
            is_left,
            is_left ? LEFT_CLOSE : RIGHT_CLOSE);
    }

    /**
     * @brief Publish one Inspire hand command.
     *
     * Called from G1Deploy's command writer thread.
     */
    void writeOnce()
    {
        if (!publisher_)
        {
            return;
        }

        Dex3Command left;
        Dex3Command right;

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            left = left_dex3_command_;
            right = right_dex3_command_;
        }

        const double max_close_ratio =
            max_close_ratio_.load(
                std::memory_order_relaxed);

        const InspireCommand left_inspire =
            dex3ToInspire(
                left,
                true,
                max_close_ratio);

        const InspireCommand right_inspire =
            dex3ToInspire(
                right,
                false,
                max_close_ratio);

        // dfx_inspire_service convention:
        //   0..5  = right hand
        //   6..11 = left hand
        for (int i = 0; i < INSPIRE_DOF; ++i)
        {
            dds_command_.cmds()[i].q() =
                right_inspire[i];

            dds_command_.cmds()[i + INSPIRE_DOF].q() =
                left_inspire[i];
        }

        publisher_->Write(dds_command_);
    }

    /**
     * @brief Return Dex3-compatible hand state.
     *
     * Inspire has 6 DOF while the rest of GEAR-SONIC currently expects
     * a 7-DOF Dex3 state. For the first Inspire integration, return the
     * last commanded Dex3-compatible target.
     *
     * Actual Inspire feedback is still subscribed and can be obtained via
     * getActualInspireState().
     */
    Dex3Command getCompatibleState(bool is_left) const
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        return is_left
            ? left_dex3_command_
            : right_dex3_command_;
    }

    /**
     * @brief Return actual 6-DOF Inspire state.
     *
     * @return {valid, state}
     */
    std::pair<bool, InspireCommand>
    getActualInspireState(bool is_left) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        InspireCommand state{};
        const int offset = is_left ? INSPIRE_DOF : 0;

        for (int i = 0; i < INSPIRE_DOF; ++i)
        {
            state[i] =
                actual_inspire_state_[offset + i];
        }

        return {has_actual_state_, state};
    }

private:
    /**
     * @brief Convert a Dex3 joint position to [0, 1] closure ratio.
     *
     * 0 = open
     * 1 = maximally closed in that joint's commanded direction.
     */
    static double normalizeClosure(
        double q,
        double min_limit,
        double max_limit)
    {
        if (q > 0.0)
        {
            if (max_limit <= 0.0)
            {
                return 0.0;
            }

            return std::clamp(
                q / max_limit,
                0.0,
                1.0);
        }

        if (q < 0.0)
        {
            if (min_limit >= 0.0)
            {
                return 0.0;
            }

            // q and min_limit are both negative.
            return std::clamp(
                q / min_limit,
                0.0,
                1.0);
        }

        return 0.0;
    }

    /**
     * @brief Convert one Dex3 7-DOF hand command to Inspire 6-DOF.
     *
     * Dex3:
     *   0 thumb_0
     *   1 thumb_1
     *   2 thumb_2
     *   3 index_0
     *   4 index_1
     *   5 middle_0
     *   6 middle_1
     *
     * Inspire:
     *   0 pinky
     *   1 ring
     *   2 middle
     *   3 index
     *   4 thumb_bend
     *   5 thumb_rotate
     *
     * Dex3 has no independent ring/pinky command in this 7-DOF
     * representation, so both follow the middle finger.
     */
    static InspireCommand dex3ToInspire(
        const Dex3Command& q,
        bool is_left,
        double max_close_ratio)
    {
        static constexpr Dex3Command MIN_LIMITS_LEFT = {
            -1.05,
            -0.724,
            0.0,
            -1.57,
            -1.75,
            -1.57,
            -1.75
        };

        static constexpr Dex3Command MAX_LIMITS_LEFT = {
            1.05,
            1.05,
            1.75,
            0.0,
            0.0,
            0.0,
            0.0
        };

        static constexpr Dex3Command MIN_LIMITS_RIGHT = {
            -1.05,
            -1.05,
            -1.75,
            0.0,
            0.0,
            0.0,
            0.0
        };

        static constexpr Dex3Command MAX_LIMITS_RIGHT = {
            1.05,
            0.742,
            0.0,
            1.57,
            1.75,
            1.57,
            1.75
        };

        const Dex3Command& min_limits =
            is_left
                ? MIN_LIMITS_LEFT
                : MIN_LIMITS_RIGHT;

        const Dex3Command& max_limits =
            is_left
                ? MAX_LIMITS_LEFT
                : MAX_LIMITS_RIGHT;

        Dex3Command closure{};

        for (int i = 0; i < DEX3_DOF; ++i)
        {
            closure[i] =
                normalizeClosure(
                    q[i],
                    min_limits[i],
                    max_limits[i]);
        }

        // Dex3 -> Inspire semantic mapping.
        const double thumb_rotate =
            closure[0];

        const double thumb_bend =
            0.5 * (closure[1] + closure[2]);

        const double index =
            0.5 * (closure[3] + closure[4]);

        const double middle =
            0.5 * (closure[5] + closure[6]);

        // Dex3's 7-DOF representation has no ring/pinky joints.
        const double ring = middle;
        const double pinky = middle;

        // Inspire:
        //   1.0 = open
        //   0.0 = closed
        const auto to_inspire =
            [max_close_ratio](double closure_ratio)
            {
                closure_ratio =
                    std::clamp(
                        closure_ratio,
                        0.0,
                        1.0);

                return std::clamp(
                    1.0 -
                        max_close_ratio *
                            closure_ratio,
                    0.0,
                    1.0);
            };

        return {
            to_inspire(pinky),
            to_inspire(ring),
            to_inspire(middle),
            to_inspire(index),
            to_inspire(thumb_bend),
            to_inspire(thumb_rotate)
        };
    }

private:
    unitree::robot::ChannelPublisherPtr<
        unitree_go::msg::dds_::MotorCmds_>
        publisher_;

    unitree::robot::ChannelSubscriberPtr<
        unitree_go::msg::dds_::MotorStates_>
        subscriber_;

    // Last command in the original Dex3 representation.
    mutable std::mutex command_mutex_;
    Dex3Command left_dex3_command_{};
    Dex3Command right_dex3_command_{};

    // Actual Inspire feedback.
    mutable std::mutex state_mutex_;
    std::array<double, TOTAL_INSPIRE_DOF>
        actual_inspire_state_{};
    bool has_actual_state_ = false;

    // DDS command reused by the 500-Hz writer thread.
    unitree_go::msg::dds_::MotorCmds_
        dds_command_;

    std::atomic<double>
        max_close_ratio_{1.0};
};

#endif  // INSPIRE_HANDS_HPP
