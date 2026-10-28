#pragma once

// High-level robot client: FSM handshakes, target streams and state
// queries, built exclusively on the comm layer. No zenoh objects leak out
// of this class.
//
// Lifecycle: a robot either opens a session of its own (the three
// constructors that take connection settings) or borrows one that is
// already open (the shared_ptr constructor). An opened session lives
// exactly as long as the robot; a borrowed one is never closed here -
// whoever opened it owns it, and it lives until its last holder releases it
// (see ZenohSession). Construction does not throw either way: check Ready()
// afterwards, then LastError() for why not.
//
// Threading: Enable/Enable(mode)/Stop/SetMode/GetRobotState block the calling
// thread (client and condition-variable waits) and must not run on zenoh
// session threads. The Send* methods are thread-safe, but only one control
// thread should stream targets at a time. One failure value names the call
// that returned it only while the comm object behind it serves no other call
// at that moment: that is what the single-control-thread rule gives the target
// streams, and why the blocking methods are meant to be driven one call at a
// time per object (two GetRobotState calls share one client; Enable,
// Enable(mode), Stop and SetMode share the run_command client). Callbacks
// passed to the Set*Callback hooks run on zenoh session threads: do not block
// and do not call the blocking methods from them. SetNamespace rebuilds all
// comm objects: call it only with no other operation on this robot in flight, and
// never from zenoh session threads.

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
inline constexpr const char* kTargetCsp = "arm_control_node/target_joint_csp";
inline constexpr const char* kTargetPosition = "arm_control_node/target_joint_position";
inline constexpr const char* kTargetMit = "arm_control_node/target_joint_mit";
inline constexpr const char* kTargetGripper = "arm_control_node/target_gripper";
inline constexpr const char* kTargetBody = "arm_control_node/target_body";
inline constexpr const char* kServiceGetState = "arm_control_node/get_state";
// The robot's only source of control modes (enable / disable / set_mode).
inline constexpr const char* kServiceRunCommand = "arm_control_node/run_command";
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
    // Connects to the robot at the factory defaults (kDefaultRobotAddress,
    // kDefaultNamespace) on a session of its own. Right for a robot that
    // still carries its factory settings and shares the network with no
    // other robot: after shipment either half can differ per robot, and
    // then both have to be given.
    Robot();
    // Same, with an explicit address and namespace. Both are required -
    // this pair is what identifies one robot.
    Robot(const std::string& address, const std::string& namespace_);
    // Opens a session of its own from `cfg`, which lives as long as this
    // robot.
    explicit Robot(const comm::ZenohConfig& cfg);
    // Creates all comm objects on the caller's session, which must be open.
    // The handle is kept for the robot's lifetime and is not closed by it:
    // the session outlives this robot even if the caller drops its own
    // handle.
    explicit Robot(std::shared_ptr<comm::ZenohSession> session);
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    bool Ready() const { return ready_; }

    // Rebuilds every comm object under a new namespace prefix (multi-robot
    // handover in one process) and clears the telemetry cache. Fails
    // without switching on an invalid namespace. Call only with no other
    // operation in flight; see the class comment for threading rules.
    Result SetNamespace(const std::string& namespace_);

    // FSM handshakes: send the run_command request, then wait for the
    // matching fsm_state. The value reports this call: "not ready", a
    // transport failure, a request the robot refused (the message carries
    // its code and text), or a wait that timed out.
    //
    // Enable() asks for the damping hold. The arm enters its mode states
    // only through SetMode(), and only from ENABLED or another mode state -
    // STOP leaves only by Enable(). While a mode is running, an enable that
    // would pull the control law out from under a moving arm is answered
    // rejected_moving; Enable() resends it until the timeout, so returning
    // success still means the robot is in ENABLED.
    Result Enable(std::chrono::milliseconds timeout = kEnableTimeout);
    Result Stop(std::chrono::milliseconds timeout = kStopTimeout);
    // Switches the control mode. The robot queues the switch until the
    // current motion settles, then moves to the mode's state; this call
    // waits for that state. In STOP the robot answers busy: SetMode never
    // enables the arm - call Enable() first. The switch itself stops the
    // drives, writes their control mode and re-enables them (a drive only
    // accepts a new mode while disabled), so the arm briefly loses torque
    // while the robot runs that sequence.
    Result SetMode(ControlMode mode, std::chrono::milliseconds timeout = kModeSwitchTimeout);
    // Same edge as Enable(), with a mode token in the request. The robot
    // does not read the mode field of `enable` (its receipt says the mode
    // was ignored), so this call lands in ENABLED as well - the damping
    // hold runs in the robot's own default drive mode. Entering MIT, CSP or
    // PP still takes SetMode(mode).
    Result Enable(ControlMode mode, std::chrono::milliseconds timeout = kEnableTimeout);

    // Target streams (best_effort, newest wins). The value reports this
    // frame: "not ready", or the publisher's own code and message - the
    // latter describes this frame under the class's streaming rule.
    // Two robot-side filters decide whether a frame acts. The topic has to
    // exist: the robot subscribes a joint target topic only when any of its
    // arm components reports that capability, and frames to an unsubscribed
    // topic are dropped silently. The state has to match: JointMITTarget is
    // accepted only in MIT, JointCSPTarget only in CSP, JointPositionTarget
    // only in PP, and then only if the drives really are in that mode; a
    // dropped frame is reported in the robot log. ENABLED and STOP accept no
    // joint targets. Sending a target never changes the mode. Gripper targets
    // act in the three mode states; body targets depend on those components
    // being configured, and the body component applies its own state rules on
    // top of that.
    Result SendJointMITTarget(const msg::JointMITTarget& target);
    Result SendJointCSPTarget(const msg::JointCSPTarget& target);
    Result SendJointPositionTarget(const msg::JointPositionTarget& target);
    Result SendGripperTarget(const msg::GripperTarget& target);
    Result SendBodyTarget(const msg::BodyTarget& target);

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
    // Shared tail of the four constructors: fails when session_ is missing
    // or not open, otherwise creates the comm objects and starts the
    // telemetry watchdog.
    void AttachToSession();
    // Tail of AttachToSession: creates the comm objects on the session held
    // and, when that succeeded, marks the robot ready and starts the
    // telemetry watchdog. Returns the failure of the last creation that
    // failed; the caller records it.
    Result InitComms();
    // (Re)creates all comm objects under the session's current namespace.
    // Every constructor sets the session before this runs. On failure
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
    // One run_command service call. The value reports the call itself (not
    // ready, transport failure); `receipt` carries the robot's answer, whose
    // accepted flag and code say whether the command will act.
    Result CallRunCommand(const std::string& command, const std::string& mode,
                          std::chrono::milliseconds timeout, msg::RunCommandResponse& receipt);
    // Shared body of the two Enable overloads: one `enable` request carrying
    // `mode` (a field the robot does not read), then a wait for ENABLED,
    // resending while the robot answers rejected_moving.
    Result EnableWithMode(const std::string& mode, std::chrono::milliseconds timeout);
    Result WaitForFsmState(const std::string& expected, std::chrono::milliseconds timeout);
    // One command whose accepted receipt names the state it lands in, followed
    // by WaitForFsmState; a refusal becomes a failure naming the robot's code
    // and message. The timeout covers both halves.
    Result CommandAndWait(const std::string& command, const std::string& mode,
                          const std::string& expected, std::chrono::milliseconds timeout);
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
    std::shared_ptr<comm::ZenohPublisher<msg::JointMITTarget>> mit_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointCSPTarget>> csp_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::JointPositionTarget>> pos_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::GripperTarget>> grip_pub_;
    std::shared_ptr<comm::ZenohPublisher<msg::BodyTarget>> body_pub_;
    std::shared_ptr<comm::ZenohClient<msg::GetStateRequest, msg::GetStateResponse>>
        get_state_cli_;
    std::shared_ptr<comm::ZenohClient<msg::RunCommandRequest, msg::RunCommandResponse>>
        run_command_cli_;

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
