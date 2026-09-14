#pragma once

// Builds zenoh key expressions from a namespace and a ROS name.
//
// Mapping rule (mirrors zenoh-plugin-ros2dds):
//   - ROS names lose their leading '/'
//   - the namespace (the plugin's `namespace` config, or empty) is prepended
//     as the first keyexpr segment
//   - empty segments are rejected, as are the zenoh wildcard characters
//     '?', '*' and '#' (they would silently change routing)
//
// Example: namespace "/robot1", topic "/joint_states"
//        -> "robot1/joint_states" (both leading slashes are stripped)

#include <string>

namespace shidou {

class KeyExprBuilder {
public:
    // True if namespace and topic are both valid; writes the keyexpr to out.
    bool BuildTopic(const std::string& ns, const std::string& topic, std::string& out);

    // ROS services map to zenoh Queryables with the same naming rule.
    bool BuildService(const std::string& ns, const std::string& service, std::string& out);

    // Human-readable reason for the last failure; unchanged after success.
    const std::string& LastError() const { return last_error_; }

private:
    bool BuildPath(const std::string& ns, const char* kind, const std::string& name,
                   std::string& out);

    std::string last_error_;
};

} // namespace shidou
