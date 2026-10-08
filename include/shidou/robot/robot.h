#pragma once

// High-level robot client: FSM handshakes, target streams, trajectory
// upload and state queries, built exclusively on the comm layer. No zenoh
// objects leak out of this class.
//
// Lifecycle: the constructor creates all comm objects on the session it
// receives and keeps that handle for the robot's lifetime. The robot does NOT
// close the session: whoever opened it owns it, and the session lives until
// its last holder releases it (see ZenohSession). The robot's own handle
// keeps it alive for at least as long as this robot.
//
// Threading: Enable/Stop/SetMode/UploadTrajectory/GetRobotState block the
// calling thread (condition-variable or client waits) and must not run on
// zenoh session threads. The Send* methods are thread-safe, but only one
// control thread should stream targets at a time. One failure value names the
// call that returned it only while the comm object behind it serves no other
// call at that moment: that is what the single-control-thread rule gives the
// target streams, and why the blocking methods are meant to be driven one call
// at a time per object (two GetRobotState calls share one client; Enable, Stop
// and SetMode share the fsm command publisher). Callbacks passed to the
// Set*Callback hooks run on zenoh session threads: do not block and do not call
// the blocking methods from them. SetNamespace rebuilds all comm objects: call
// it only with no other operation on this robot in flight, and never from
// zenoh session threads.

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
#include "shidou/comm/zenoh_session.h"
#include "shidou/comm/zenoh_subscriber.h"
#include "shidou/robot/defaults.h"
#include "shidou/robot/fsm.h"
#include "shidou/robot/result.h"
#include "shidou/robot/robot_state.h"
#include "shidou/robot/telemetry.h"

namespace shidou::robot {

// Topic and service names of the robot-side interface. The namespace of
// the session the robot runs on is prepended by the comm layer, so these
// stay namespace-free.
namespace topics {
inline constexpr const char* kJointStates = "arm_control_node/joint_states";
inline constexpr const char* kFsmState = "arm_control_node/fsm_state";
inline constexpr const char* kFsmCommand = "arm_control_node/fsm_command";
inline constexpr const char* kTargetCsp = "arm_control_node/target_joint_csp";
inline constexpr const char* kTargetPosition = "arm_control_node/target_joint_position";
inline constexpr const char* kTargetMit = "arm_control_node/target_joint_mit";
inline constexpr const char* kTargetGripper = "arm_control_node/target_gripper";
inline constexpr const char* kTargetBody = "arm_control_node/target_body";
inline constexpr const char* kServiceGetState = "arm_control_node/get_state";
inline constexpr const char* kServiceJointTrajectory = "arm_control_node/joint_trajectory";
} // namespace topics

// Identity of one target stream: the topic it goes to and the reliability
// it is delivered with (see comm/options.h). Robot creates each target
// publisher from its spec row, so the topic and the qos cannot drift apart
// between creation and use. The type parameter binds the row to its
// message type: rows are not interchangeable.
template <typename T>
struct TargetSpec {
    const char* topic;
    comm::Reliability qos;
};

inline constexpr TargetSpec<msg::JointMITTarget> kTargetMitSpec{topics::kTargetMit,
                                                                comm::Reliability::kBestEffort};
inline constexpr TargetSpec<msg::JointCSPTarget> kTargetCspSpec{topics::kTargetCsp,
                                                                comm::Reliability::kBestEffort};
inline constexpr TargetSpec<msg::JointPositionTarget> kTargetPositionSpec{
    topics::kTargetPosition, comm::Reliability::kBestEffort};
inline constexpr TargetSpec<msg::GripperTarget> kTargetGripperSpec{topics::kTargetGripper,
                                                                   comm::Reliability::kBestEffort};
inline constexpr TargetSpec<msg::BodyTarget> kTargetBodySpec{topics::kTargetBody,
                                                             comm::Reliability::kBestEffort};

class Robot {
public:
    // Creates all comm objects on the caller's session, which must be open.
    // The handle is kept for the robot's lifetime and is not closed by it:
    // the session outlives this robot even if the caller drops its own
    // handle. Check Ready() afterwards; on failure see LastError().
    explicit Robot(std::shared_ptr<comm::ZenohSession> session);
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    bool Ready() const { return ready_; }

    // Rebuilds every comm object under a new namespace prefix (multi-robot
    // handover in one process) and clears the telemetry cache. Fails
    // without switching on an invalid namespace. Call only with no other
    // operation in flight; see the class comment for threading rules.
    Result SetNamespace(const std::string& ns);

    // FSM handshake: publish the command, then wait for the matching
    // fsm_state. The value reports this call: "not ready", or a wait that
    // timed out. STOP only accepts Enable(); mode commands require the
    // ENABLED state (they are ignored by the robot otherwise).
    Result Enable(std::chrono::milliseconds timeout = kEnableTimeout);
    Result Stop(std::chrono::milliseconds timeout = kStopTimeout);
    Result SetMode(ControlMode mode, std::chrono::milliseconds timeout = kModeSwitchTimeout);

    // Target streams (best_effort, newest wins). The value reports this
    // frame: "not ready", or the publisher's own code and message - the
    // latter describes this frame under the class's streaming rule.
    Result SendJointMITTarget(const msg::JointMITTarget& target);
    Result SendJointCSPTarget(const msg::JointCSPTarget& target);
    Result SendJointPositionTarget(const msg::JointPositionTarget& target);
    Result SendGripperTarget(const msg::GripperTarget& target);
    Result SendBodyTarget(const msg::BodyTarget& target);

    // Uploads a trajectory through the joint_trajectory service. The value
    // reports this call: "not ready", the client's failure, or the robot
    // rejecting the trajectory. Run it with SetMode(kTrajectory)
    // afterwards; the robot leaves TRAJECTORY when execution finishes.
    Result UploadTrajectory(const msg::JointTrajectory& trajectory,
                            std::chrono::milliseconds timeout = kTrajectoryUploadTimeout);

    // get_state service call merged with the telemetry cache snapshot. The
    // value reports this call: "not ready", or the client's failure.
    Result GetRobotState(RobotState& out, std::chrono::milliseconds timeout = kGetStateTimeout);

    // Telemetry hooks; see the class comment for threading rules.
    void SetJointFeedbackCallback(std::function<void(const msg::JointFeedback&)> cb);
    void SetFsmStateCallback(std::function<void(const std::string&)> cb);
    void SetFeedbackStaleCallback(std::chrono::milliseconds age_threshold_ms,
                                  std::function<void(double age_ms)> cb);

    // Telemetry snapshot accessors (safe from any thread; a namespace
    // switch excludes all concurrent calls, as stated in the class
    // comment). They read the subscriber sample caches, the single owner of
    // the latest samples.
    bool LastFeedback(msg::JointFeedback& out) const {
        return feedback_sub_ != nullptr && feedback_sub_->LastValue(out);
    }
    double FeedbackAgeMs() const {
        if (feedback_sub_ == nullptr || feedback_sub_->Seq() == 0) {
            return -1.0;  // negative until the first sample
        }
        return feedback_sub_->AgeMs();
    }
    uint64_t FeedbackSeq() const {
        return feedback_sub_ == nullptr ? 0 : feedback_sub_->Seq();
    }
    std::string FsmState() const {
        msg::String state;
        if (fsm_sub_ == nullptr || !fsm_sub_->LastValue(state)) {
            return {};
        }
        return state.data;
    }

    // Diagnostic record of the most recent failure on this robot: the code
    // and the message of that failure, whichever operation produced it.
    // A success does not clear it, so it is not the outcome of the last call -
    // read the value that call returned for that. Copies, not synchronized
    // with concurrent operations.
    ErrorCode LastErrorCode() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_code_;
    }
    std::string LastError() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

private:
    // Tail of the constructor: creates the comm objects on the session held
    // and, when that succeeded, marks the robot ready and starts the
    // telemetry watchdog. Returns the failure of the last creation that
    // failed; the caller records it.
    Result InitComms();
    // (Re)creates all comm objects under the session's current namespace.
    // The session is set by both constructors before this runs. On failure
    // the members are a mix of new and old objects; callers mark the robot
    // not-ready in that case. The value carries the session's code and
    // message: every creation is attempted, so with more than one failure
    // that record belongs to the last one that failed.
    Result CreateCommObjects();
    // One target stream publish, the shape all five Send* share. The
    // publisher's record read after a failed publish is this call's under the
    // single-control-thread rule (see the class comment); see the definition
    // for the failure it reports.
    template <typename T>
    Result SendTarget(comm::ZenohPublisher<T>& pub, const T& target);
    Result SendFsmCommand(const std::string& command);
    Result WaitForFsmState(const std::string& expected, std::chrono::milliseconds timeout);
    Result CommandAndWait(const std::string& command, const std::string& expected,
                          std::chrono::milliseconds timeout);
    void OnFeedback(const msg::JointFeedback& fb);  // zenoh session thread
    void OnFsmState(const msg::String& state);      // zenoh session thread
    // The value of every operation that runs while the robot is not ready:
    // one wording for one condition, so a caller can tell "called too early"
    // from a link failure by code as well as by text.
    Result NotReady();
    // Records the failure in the sticky diagnostic, logs it, and hands it
    // back as the value of the operation that is about to return it.
    Result Fail(ErrorCode code, std::string message);

    bool ready_ = false;
    // Declared before the comm objects so it is destroyed after them: the
    // session object outlives what it created. Its session-lost callback is
    // what the comm objects report to, and they may outlive even this handle.
    std::shared_ptr<comm::ZenohSession> session_;
    std::shared_ptr<comm::ZenohSubscriber<msg::JointFeedback>> feedback_sub_;
    std::shared_ptr<comm::ZenohSubscriber<msg::String>> fsm_sub_;
    std::shared_ptr<comm::ZenohPublisher<msg::String>> fsm_cmd_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointMITTarget>> mit_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointCSPTarget>> csp_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointPositionTarget>> pos_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::GripperTarget>> grip_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::BodyTarget>> body_pub_;
    std::shared_ptr<comm::ZenohClient<msg::GetStateRequest, msg::GetStateResponse>>
        get_state_cli_;
    std::shared_ptr<comm::ZenohClient<msg::JointTrajectoryRequest, msg::JointTrajectoryResponse>>
        traj_cli_;

    Telemetry telemetry_;
    // Dedicated mutex for fsm_cv_: the predicate reads the fsm_state sample
    // cache under its own lock, so this mutex guards no shared data.
    std::mutex fsm_wait_mutex_;
    std::condition_variable fsm_cv_;
    mutable std::mutex error_mutex_;
    ErrorCode last_error_code_ = ErrorCode::kOk;
    std::string last_error_;
};

} // namespace shidou::robot
