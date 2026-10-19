#include <cmath>
#include <chrono>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        this->declare_parameter("scan_angle", 20);           // half-angle in degrees
        this->declare_parameter("obstacle_distance", 0.5);   // metres
        this->declare_parameter("forward_speed", 0.1);       // m/s (Burger max 0.22 m/s)
        this->declare_parameter("turn_speed", 0.3);          // rad/s (Burger max 2.84 rad/s)
        this->declare_parameter("enabled", true);
        parameter_callback_ = add_on_set_parameters_callback(
            [](const std::vector<rclcpp::Parameter> & params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & p : params) {
                    if (p.get_name() == "scan_angle") {
                        result.successful = p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                            p.as_int() > 0 && p.as_int() <= 180;
                    } else if (p.get_name() == "forward_speed" || p.get_name() == "turn_speed" ||
                               p.get_name() == "obstacle_distance") {
                        result.successful = p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                            std::isfinite(p.as_double()) && p.as_double() > 0;
                    }
                    if (!result.successful) { result.reason = "Invalid positive speed/distance or angle (1..180)"; break; }
                }
                return result;
            });
        watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            if (!get_parameter("enabled").as_bool() ||
                (std::chrono::steady_clock::now() - last_scan_) > std::chrono::seconds(1)) {
                cmd_pub_->publish(geometry_msgs::msg::Twist());
            }
        });
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        const int SCAN_ANGLE = this->get_parameter("scan_angle").as_int();
        const double OBSTACLE_DISTANCE = this->get_parameter("obstacle_distance").as_double();
        const double FORWARD_SPEED = this->get_parameter("forward_speed").as_double();
        const double TURN_SPEED = this->get_parameter("turn_speed").as_double();

        // Normalize scan bearings to [-pi, pi].
        last_scan_ = std::chrono::steady_clock::now();
        bool isObstacleDetectedL = false, isObstacleDetectedR = false;
        float minL = msg->range_max, minR = msg->range_max;
        const double half_angle = SCAN_ANGLE * std::acos(-1.0) / 180.0;
        if (!get_parameter("enabled").as_bool() || msg->ranges.empty() ||
            !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) ||
            msg->angle_increment == 0 || !std::isfinite(msg->range_min) ||
            !std::isfinite(msg->range_max) || msg->range_min < 0 ||
            msg->range_max <= msg->range_min || SCAN_ANGLE <= 0 || SCAN_ANGLE > 180 ||
            !std::isfinite(OBSTACLE_DISTANCE) || OBSTACLE_DISTANCE <= 0 ||
            !std::isfinite(FORWARD_SPEED) || FORWARD_SPEED <= 0 ||
            !std::isfinite(TURN_SPEED) || TURN_SPEED <= 0) {
            cmd_pub_->publish(cmd_msg);
            return;
        }
        size_t usable = 0;
        for (size_t index = 0; index < msg->ranges.size(); ++index) {
            const double raw = msg->angle_min + index * static_cast<double>(msg->angle_increment);
            const double angle = std::atan2(std::sin(raw), std::cos(raw));
            if (std::abs(angle) > half_angle + 1e-6) continue;
            const float range = msg->ranges[index];
            // Positive infinity indicates no return.
            if (std::isinf(range) && range > 0) { ++usable; continue; }
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) continue;
            ++usable;
            if (angle >= -1e-6) {
                minL = std::min(minL, range);
                if (range < OBSTACLE_DISTANCE) isObstacleDetectedL = true;
            }
            if (angle <= 1e-6) {
                minR = std::min(minR, range);
                if (range < OBSTACLE_DISTANCE) isObstacleDetectedR = true;
            }
        }
        if (usable == 0) { cmd_pub_->publish(cmd_msg); return; }

        if (isObstacleDetectedL)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -TURN_SPEED; // obstacle on the left -> spin right
        }
        else if (isObstacleDetectedR)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = TURN_SPEED;  // obstacle on the right -> spin left
        }
        else
        {
            cmd_msg.linear.x = FORWARD_SPEED; // move forward
            cmd_msg.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd_msg);

        const char * action = isObstacleDetectedL ? "SPIN RIGHT" :
                              isObstacleDetectedR ? "SPIN LEFT" : "FORWARD";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "L: %.2f m | R: %.2f m | %-10s | v=%.2f w=%.2f | angle=%d thr=%.2f",
            minL, minR, action, cmd_msg.linear.x, cmd_msg.angular.z,
            SCAN_ANGLE, OBSTACLE_DISTANCE);
    }

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::chrono::steady_clock::time_point last_scan_ = std::chrono::steady_clock::now();
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoPilot>());
    rclcpp::shutdown();
    return 0;
}
