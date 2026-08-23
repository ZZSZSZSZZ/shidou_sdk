#pragma once

// High-level robot client: FSM handshakes, target streams, trajectory
// upload and state queries, built exclusively on the comm layer. No zenoh
// objects leak out of this class.
//
// Lifecycle: the constructor initializes the ZenohFactory singleton from
// the config and creates all comm objects. The factory is NOT shut down
// here: callers own the process-level lifecycle (Init once / Shutdown at
// process end, including after destroying this robot).
//
// Threading: Enable/Stop/SetMode/UploadTrajectory/GetRobotState block the
// calling thread (condition-variable or client waits) and must not run on
// zenoh session threads. The Send* methods are thread-safe, but only one
// control thread should stream targets at a time. Callbacks passed to the
// Set*Callback hooks run on zenoh session threads: do not block and do
// not call the blocking methods from them. SetNamespace rebuilds all comm
// objects: call it only with no other operation on this robot in flight,
// and never from zenoh session threads.

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "shidou/codec/msg_codecs.h"
#include "shidou/comm/options.h"
#include "shidou/comm/zenoh_client.h"
#include "shidou/comm/zenoh_publisher.h"
#include "shidou/comm/zenoh_subscriber.h"
#include "shidou/robot/defaults.h"
#include "shidou/robot/fsm.h"
#include "shidou/robot/robot_state.h"
#include "shidou/robot/telemetry.h"

namespace shidou::robot {

// Topic and service names of the robot-side interface. The namespace
// config of the comm layer is prepended by the factory, so these stay
// namespace-free.
namespace topics {
inline constexpr const char* kJointStates = "arm_control_node/joint_states";
inline constexpr const char* kFsmState = "arm_control_node/fsm_state";
inline constexpr const char* kFsmCommand = "arm_control_node/fsm_command";
inline constexpr const char* kTargetCsp = "arm_control_node/target_joint_csp";
inline constexpr const char* kTargetPosition = "arm_control_node/target_joint_position";
inline constexpr const char* kTargetMit = "arm_control_node/target_joint_mit";
inline constexpr const char* kTargetGripper = "arm_control_node/target_gripper";
inline constexpr const char* kServiceGetState = "arm_control_node/get_state";
inline constexpr const char* kServiceJointTrajectory = "arm_control_node/joint_trajectory";
} // namespace topics

class Robot {
public:
    // Initializes the comm layer and creates all comm objects. Check
    // Ready() afterwards; on failure see LastError().
    explicit Robot(const comm::ZenohConfig& cfg);
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    bool Ready() const { return ready_; }

    // Rebuilds every comm object under a new namespace prefix (multi-robot
    // handover in one process) and clears the telemetry cache. Fails
    // without switching on an invalid namespace. Call only with no other
    // operation in flight; see the class comment for threading rules.
    bool SetNamespace(const std::string& ns);

    // FSM handshake: publish the command, then wait for the matching
    // fsm_state. STOP only accepts Enable(); mode commands require the
    // ENABLED state (they are ignored by the robot otherwise).
    bool Enable(std::chrono::milliseconds timeout = kEnableTimeout);
    bool Stop(std::chrono::milliseconds timeout = kStopTimeout);
    bool SetMode(ControlMode mode, std::chrono::milliseconds timeout = kModeSwitchTimeout);

    // Target streams (best_effort, newest wins). Returns false when the
    // publication fails (session trouble) or the robot is not ready.
    bool SendJointMITTarget(const msg::JointMITTarget& target);
    bool SendJointCSPTarget(const msg::JointCSPTarget& target);
    bool SendJointPositionTarget(const msg::JointPositionTarget& target);
    bool SendGripperTarget(const msg::GripperTarget& target);

    // Uploads a trajectory through the joint_trajectory service and
    // returns the robot's acceptance. Run it with SetMode(kTrajectory)
    // afterwards; the robot leaves TRAJECTORY when execution finishes.
    bool UploadTrajectory(const msg::JointTrajectory& trajectory,
                          std::chrono::milliseconds timeout = kTrajectoryUploadTimeout);

    // get_state service call merged with the telemetry cache snapshot.
    bool GetRobotState(RobotState& out, std::chrono::milliseconds timeout = kGetStateTimeout);

    // Telemetry hooks; see the class comment for threading rules.
    void SetJointFeedbackCallback(std::function<void(const msg::JointFeedback&)> cb);
    void SetFsmStateCallback(std::function<void(const std::string&)> cb);
    void SetFeedbackStaleCallback(std::chrono::milliseconds age_threshold_ms,
                                  std::function<void(double age_ms)> cb);

    // Telemetry snapshot accessors (safe from any thread).
    bool LastFeedback(msg::JointFeedback& out) const { return telemetry_.LastFeedback(out); }
    double FeedbackAgeMs() const { return telemetry_.FeedbackAgeMs(); }
    uint64_t FeedbackSeq() const { return telemetry_.FeedbackSeq(); }
    std::string FsmState() const { return telemetry_.FsmState(); }

    // Last failure message; empty after a success. Copy, not synchronized
    // with concurrent operations.
    std::string LastError() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

private:
    // (Re)creates all comm objects under the factory's current namespace.
    // On failure the members are a mix of new and old objects; callers
    // mark the robot not-ready in that case.
    bool CreateCommObjects();
    bool SendFsmCommand(const std::string& command);
    bool WaitForFsmState(const std::string& expected, std::chrono::milliseconds timeout);
    bool CommandAndWait(const std::string& command, const std::string& expected,
                        std::chrono::milliseconds timeout);
    void OnFeedback(const msg::JointFeedback& fb);  // zenoh session thread
    void OnFsmState(const msg::String& state);      // zenoh session thread
    bool Fail(const std::string& message);

    bool ready_ = false;
    std::shared_ptr<comm::ZenohSubscriber<msg::JointFeedback>> feedback_sub_;
    std::shared_ptr<comm::ZenohSubscriber<msg::String>> fsm_sub_;
    std::shared_ptr<comm::ZenohPublisher<msg::String>> fsm_cmd_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointMITTarget>> mit_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointCSPTarget>> csp_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointPositionTarget>> pos_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::GripperTarget>> grip_pub_;
    std::shared_ptr<comm::ZenohClient<msg::GetStateRequest, msg::GetStateResponse>>
        get_state_cli_;
    std::shared_ptr<comm::ZenohClient<msg::JointTrajectoryRequest, msg::JointTrajectoryResponse>>
        traj_cli_;

    Telemetry telemetry_;
    // Dedicated mutex for fsm_cv_: the predicate reads the telemetry
    // cache under its own lock, so this mutex guards no shared data.
    std::mutex fsm_wait_mutex_;
    std::condition_variable fsm_cv_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

} // namespace shidou::robot
