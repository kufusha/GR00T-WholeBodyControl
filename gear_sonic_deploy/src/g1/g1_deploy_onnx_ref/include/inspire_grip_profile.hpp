#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace inspire_grip {

using Pose = std::array<double, 6>;
inline constexpr std::array<const char*, 6> axis_names = {
    "pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotate"
};

struct Point {
    double grip;
    Pose left;
    Pose right;
};

class ProfileError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class Profile {
public:
    static Profile fromEnvironment() {
        const char* setting = std::getenv("SONIC_INSPIRE_GRIP_PROFILE");
        const std::filesystem::path path = setting ? std::filesystem::path(setting) : defaultPath();
        try {
            const YAML::Node document = YAML::LoadFile(path.string());
            Profile profile;
            profile.points_ = readPoints(document["points"], "points", 1.0);
            if (const auto force = document["force_points"]) {
                profile.force_points_ = readPoints(force, "force_points", 1000.0);
            }
            std::clog << "[InspireHands] Grip profile: " << path.string() << '\n';
            return profile;
        } catch (const std::exception& error) {
            throw ProfileError("Invalid Inspire grip profile '" + path.string() + "': " + error.what());
        }
    }

    Pose sample(bool is_left, double grip) const {
        return interpolate(points_, is_left, grip);
    }

    std::optional<Pose> sampleForce(bool is_left, double grip) const {
        if (force_points_.empty()) return std::nullopt;
        return interpolate(force_points_, is_left, grip);
    }

private:
    Profile() = default;

    static Pose interpolate(const std::vector<Point>& points, bool is_left, double grip) {
        const auto next = std::lower_bound(points.begin(), points.end(), grip,
            [](const Point& point, double value) { return point.grip < value; });
        if (next == points.end()) {
            return is_left ? points.back().left : points.back().right;
        }
        const Pose& end = is_left ? next->left : next->right;
        if (next == points.begin() || next->grip == grip) {
            return end;
        }
        const Point& previous = *(next - 1);
        const Pose& start = is_left ? previous.left : previous.right;
        const double t = (grip - previous.grip) / (next->grip - previous.grip);
        Pose result{};
        for (std::size_t i = 0; i < result.size(); ++i) {
            result[i] = start[i] + t * (end[i] - start[i]);
        }
        return result;
    }

    static std::vector<Point> readPoints(const YAML::Node& nodes, const std::string& key, double maximum) {
        if (!nodes.IsSequence() || nodes.size() < 2) {
            throw std::runtime_error(key + " must contain at least two poses");
        }
        std::vector<Point> points;
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto node = nodes[i];
            const auto where = key + "[" + std::to_string(i) + "]";
            const double grip = number(node["grip"], where + ".grip", 1.0);
            if (!points.empty() && grip <= points.back().grip) {
                throw std::runtime_error(where + ".grip must increase strictly");
            }
            points.push_back({grip, pose(node["left"], where + ".left", maximum),
                                    pose(node["right"], where + ".right", maximum)});
        }
        if (points.front().grip != 0.0 || points.back().grip != 1.0) {
            throw std::runtime_error(key + ".grip must start at 0 and end at 1");
        }
        return points;
    }

    static std::filesystem::path defaultPath() {
        const std::filesystem::path relative("config/inspire/grip3.yml");
        std::error_code error;
        const auto directory = std::filesystem::read_symlink("/proc/self/exe", error).parent_path();
        if (!error && directory.filename() == "release" && directory.parent_path().filename() == "target") {
            return directory.parent_path().parent_path() / relative;
        }
        return relative;
    }

    static double number(const YAML::Node& node, const std::string& where, double maximum) {
        if (!node.IsScalar() || node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") {
            throw std::runtime_error(where + " must be a number");
        }
        const double value = node.as<double>(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(value) || value < 0.0 || value > maximum) {
            throw std::runtime_error(where + " must be finite and within 0.." + std::to_string(int(maximum)));
        }
        return value;
    }

    static Pose pose(const YAML::Node& values, const std::string& where, double maximum) {
        Pose result{};
        if ((!values.IsMap() && !values.IsSequence()) || values.size() != result.size()) {
            throw std::runtime_error(where + " must contain exactly six axes");
        }
        for (std::size_t i = 0; i < result.size(); ++i) {
            const auto name = axis_names[i];
            const auto node = values.IsMap() ? values[name] : values[i];
            result[i] = number(node, where + "." + name, maximum);
        }
        return result;
    }

    std::vector<Point> points_;
    std::vector<Point> force_points_;
};

}  // namespace inspire_grip
