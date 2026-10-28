#pragma once

// Encode/Decode implementations for every message in shidou::msg.
//
// Field order here is the wire contract: it must match the robot-side ROS
// message definitions exactly. Change both sides together or never.
// Primitive layout follows codec/cdr.h.

#include "shidou/codec/cdr.h"
#include "shidou/msg/body_target.h"
#include "shidou/msg/get_state.h"
#include "shidou/msg/gripper_target.h"
#include "shidou/msg/joint_csp_target.h"
#include "shidou/msg/joint_feedback.h"
#include "shidou/msg/joint_mit_target.h"
#include "shidou/msg/joint_position_target.h"
#include "shidou/msg/run_command.h"
#include "shidou/msg/string_msg.h"

namespace shidou::msg {

inline bool String::Encode(codec::WriteBuffer& out) const {
    return codec::WriteString(out, data);
}

inline bool String::Decode(codec::ReadCursor& in) {
    return codec::ReadString(in, data);
}

inline bool JointFeedback::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, position) &&
           codec::WriteSeq(out, velocity) && codec::WriteSeq(out, effort) &&
           codec::WriteSeq(out, enabled) && codec::WriteSeq(out, online) &&
           codec::WriteSeq(out, fault_code) && codec::WriteSeq(out, temperature);
}

inline bool JointFeedback::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, position) &&
           codec::ReadSeq(in, velocity) && codec::ReadSeq(in, effort) &&
           codec::ReadSeq(in, enabled) && codec::ReadSeq(in, online) &&
           codec::ReadSeq(in, fault_code) && codec::ReadSeq(in, temperature);
}

inline bool JointCSPTarget::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, positions) &&
           codec::WriteSeq(out, velocities) && codec::WriteSeq(out, torques);
}

inline bool JointCSPTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, positions) &&
           codec::ReadSeq(in, velocities) && codec::ReadSeq(in, torques);
}

inline bool JointPositionTarget::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, positions) &&
           codec::WriteSeq(out, velocities) && codec::WriteSeq(out, torques) &&
           codec::WriteSeq(out, accelerations);
}

inline bool JointPositionTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, positions) &&
           codec::ReadSeq(in, velocities) && codec::ReadSeq(in, torques) &&
           codec::ReadSeq(in, accelerations);
}

inline bool JointMITTarget::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, positions) &&
           codec::WriteSeq(out, velocities) && codec::WriteSeq(out, torques) &&
           codec::WriteSeq(out, kps) && codec::WriteSeq(out, kds);
}

inline bool JointMITTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, positions) &&
           codec::ReadSeq(in, velocities) && codec::ReadSeq(in, torques) &&
           codec::ReadSeq(in, kps) && codec::ReadSeq(in, kds);
}

inline bool BodyTarget::Encode(codec::WriteBuffer& out) const {
    // pushrod_id leaves the cursor 4-aligned, so position (the first 8-byte
    // member) goes through the aligned helper: 4 bytes of padding whenever
    // the sequences above end on a 4-mod-8 offset. The aligned helpers also
    // cover the first element of each non-empty sequence, whose length
    // prefix can leave the cursor misaligned the same way.
    return codec::WriteSeq(out, wheel_ids) && codec::WriteSeq(out, velocities) &&
           codec::WriteSeq(out, max_currents) && codec::WriteU32Aligned(out, pushrod_id) &&
           codec::WriteF64Aligned(out, position) && codec::WriteF64Aligned(out, velocity) &&
           codec::WriteF64Aligned(out, acceleration);
}

inline bool BodyTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, wheel_ids) && codec::ReadSeq(in, velocities) &&
           codec::ReadSeq(in, max_currents) && codec::ReadU32Aligned(in, pushrod_id) &&
           codec::ReadF64Aligned(in, position) && codec::ReadF64Aligned(in, velocity) &&
           codec::ReadF64Aligned(in, acceleration);
}

inline bool GripperTarget::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, open) &&
           codec::WriteSeq(out, kps) && codec::WriteSeq(out, kds);
}

inline bool GripperTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, open) &&
           codec::ReadSeq(in, kps) && codec::ReadSeq(in, kds);
}

inline bool RunCommandRequest::Encode(codec::WriteBuffer& out) const {
    return codec::WriteString(out, command) && codec::WriteString(out, mode);
}

inline bool RunCommandRequest::Decode(codec::ReadCursor& in) {
    return codec::ReadString(in, command) && codec::ReadString(in, mode);
}

inline bool RunCommandResponse::Encode(codec::WriteBuffer& out) const {
    // accepted is one byte; WriteString aligns to 4, so it lands after the
    // padding ROS serialization puts between a bool and a string field.
    return codec::WriteBool(out, accepted) && codec::WriteString(out, code) &&
           codec::WriteString(out, message);
}

inline bool RunCommandResponse::Decode(codec::ReadCursor& in) {
    return codec::ReadBool(in, accepted) && codec::ReadString(in, code) &&
           codec::ReadString(in, message);
}

inline bool GetStateRequest::Encode(codec::WriteBuffer& out) const {
    (void)out;
    return true;  // empty request: zero-byte CDR body
}

inline bool GetStateRequest::Decode(codec::ReadCursor& in) {
    (void)in;
    return true;  // strict callers require zero bytes via DecodeFromVector
}

inline bool GetStateResponse::Encode(codec::WriteBuffer& out) const {
    if (!(codec::WriteString(out, fsm_state) && codec::WriteString(out, arm_info) &&
          codec::WriteString(out, gripper_info) && codec::WriteSeq(out, motor_ids) &&
          codec::WriteSeq(out, positions) && codec::WriteSeq(out, velocities) &&
          codec::WriteSeq(out, torques) && codec::WriteU32Aligned(out, library_status) &&
          codec::WriteF64Aligned(out, control_freq_hz))) {
        return false;
    }
    // The three doubles are contiguous once the first is 8-aligned.
    out.WriteF64(control_cycle_avg_ms);
    out.WriteF64(control_cycle_max_ms);
    return codec::WriteU32Aligned(out, cycle_overruns) && codec::WriteString(out, model) &&
           codec::WriteString(out, body_info) && codec::WriteSeqString(out, control_mode) &&
           codec::WriteSeqString(out, control_mode_confidence);
}

inline bool GetStateResponse::Decode(codec::ReadCursor& in) {
    if (!(codec::ReadString(in, fsm_state) && codec::ReadString(in, arm_info) &&
          codec::ReadString(in, gripper_info) && codec::ReadSeq(in, motor_ids) &&
          codec::ReadSeq(in, positions) && codec::ReadSeq(in, velocities) &&
          codec::ReadSeq(in, torques) && codec::ReadU32Aligned(in, library_status) &&
          codec::ReadF64Aligned(in, control_freq_hz) && in.ReadF64(control_cycle_avg_ms) &&
          in.ReadF64(control_cycle_max_ms) && codec::ReadU32Aligned(in, cycle_overruns))) {
        return false;
    }
    // Additive tail: model, body_info, control_mode and control_mode_confidence
    // were appended to the reply in that order, so a robot may end the body
    // at the last statistic (predating all four) or stop after any of them.
    // Padding the body to a 4-byte boundary adds at most 3 zero bytes, and
    // both a string and a sequence start with a 4-byte length word, so fewer
    // bytes left than that is padding, not a field, and ends the tail.
    // Whatever is left unread after this is still subject to the frame's own
    // trailing-byte rule (at most 3 zero bytes).
    if (in.Remaining() >= codec::kLengthPrefixBytes && !codec::ReadString(in, model)) {
        return false;
    }
    if (in.Remaining() >= codec::kLengthPrefixBytes && !codec::ReadString(in, body_info)) {
        return false;
    }
    if (in.Remaining() >= codec::kLengthPrefixBytes && !codec::ReadSeqString(in, control_mode)) {
        return false;
    }
    if (in.Remaining() >= codec::kLengthPrefixBytes &&
        !codec::ReadSeqString(in, control_mode_confidence)) {
        return false;
    }
    return true;
}

} // namespace shidou::msg
