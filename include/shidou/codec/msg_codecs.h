#pragma once

// Encode/Decode implementations for every message in shidou::msg.
//
// Field order here is the wire contract: it must match the robot-side ROS
// message definitions exactly. Change both sides together or never.
// Primitive layout follows codec/cdr.h; a
// service request whose first member is a struct starts that struct at
// offset 0, so encoding the member directly is wire-identical.

#include "shidou/codec/cdr.h"
#include "shidou/msg/get_state.h"
#include "shidou/msg/gripper_target.h"
#include "shidou/msg/header.h"
#include "shidou/msg/joint_csp_target.h"
#include "shidou/msg/joint_feedback.h"
#include "shidou/msg/joint_mit_target.h"
#include "shidou/msg/joint_position_target.h"
#include "shidou/msg/joint_trajectory.h"
#include "shidou/msg/string_msg.h"
#include "shidou/msg/trajectory_srv.h"

namespace shidou::msg {

inline bool String::Encode(codec::WriteBuffer& out) const {
    return codec::WriteString(out, data);
}

inline bool String::Decode(codec::ReadCursor& in) {
    return codec::ReadString(in, data);
}

inline bool Header::Encode(codec::WriteBuffer& out) const {
    // stamp: fixed 8-byte pair, no padding; frame_id follows 4-aligned.
    out.WriteI32(stamp_sec);
    out.WriteU32(stamp_nanosec);
    return codec::WriteString(out, frame_id);
}

inline bool Header::Decode(codec::ReadCursor& in) {
    return in.ReadI32(stamp_sec) && in.ReadU32(stamp_nanosec) &&
           codec::ReadString(in, frame_id);
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

inline bool GripperTarget::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, motor_ids) && codec::WriteSeq(out, open) &&
           codec::WriteSeq(out, kps) && codec::WriteSeq(out, kds);
}

inline bool GripperTarget::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, motor_ids) && codec::ReadSeq(in, open) &&
           codec::ReadSeq(in, kps) && codec::ReadSeq(in, kds);
}

inline bool MITWaypoint::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, positions) && codec::WriteSeq(out, velocities) &&
           codec::WriteSeq(out, torques) && codec::WriteBool(out, stop_point);
}

inline bool MITWaypoint::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, positions) && codec::ReadSeq(in, velocities) &&
           codec::ReadSeq(in, torques) && codec::ReadBool(in, stop_point);
}

inline bool CSPWaypoint::Encode(codec::WriteBuffer& out) const {
    return codec::WriteSeq(out, positions) && codec::WriteSeq(out, max_velocity) &&
           codec::WriteSeq(out, max_acceleration) && codec::WriteSeq(out, torques);
}

inline bool CSPWaypoint::Decode(codec::ReadCursor& in) {
    return codec::ReadSeq(in, positions) && codec::ReadSeq(in, max_velocity) &&
           codec::ReadSeq(in, max_acceleration) && codec::ReadSeq(in, torques);
}

inline bool JointTrajectory::Encode(codec::WriteBuffer& out) const {
    return header.Encode(out) && codec::WriteSeq(out, joint_id) &&
           codec::WriteSeqStruct(out, mit_points) && codec::WriteSeqStruct(out, csp_points);
}

inline bool JointTrajectory::Decode(codec::ReadCursor& in) {
    return header.Decode(in) && codec::ReadSeq(in, joint_id) &&
           codec::ReadSeqStruct(in, mit_points) && codec::ReadSeqStruct(in, csp_points);
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
    return codec::WriteU32Aligned(out, cycle_overruns);
}

inline bool GetStateResponse::Decode(codec::ReadCursor& in) {
    return codec::ReadString(in, fsm_state) && codec::ReadString(in, arm_info) &&
           codec::ReadString(in, gripper_info) && codec::ReadSeq(in, motor_ids) &&
           codec::ReadSeq(in, positions) && codec::ReadSeq(in, velocities) &&
           codec::ReadSeq(in, torques) && codec::ReadU32Aligned(in, library_status) &&
           codec::ReadF64Aligned(in, control_freq_hz) && in.ReadF64(control_cycle_avg_ms) &&
           in.ReadF64(control_cycle_max_ms) && codec::ReadU32Aligned(in, cycle_overruns);
}

inline bool JointTrajectoryRequest::Encode(codec::WriteBuffer& out) const {
    // First (and only) member starts at offset 0: encoding the member
    // directly is wire-identical to encoding the request struct.
    return trajectory.Encode(out);
}

inline bool JointTrajectoryRequest::Decode(codec::ReadCursor& in) {
    return trajectory.Decode(in);
}

inline bool JointTrajectoryResponse::Encode(codec::WriteBuffer& out) const {
    return codec::WriteBool(out, success);
}

inline bool JointTrajectoryResponse::Decode(codec::ReadCursor& in) {
    return codec::ReadBool(in, success);
}

} // namespace shidou::msg
