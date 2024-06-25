/**
 * Test the velocity and joystick exchange functionality
 * Calculate the relationship between SpeedProfile setting and the speed of the WHILL
 * when controlling using wheelchair 
 * Author: Le Van Tan
 * Email: levantann321@gmail.com/le.van.tan.t4@dc.tohoku.ac.jp
 * * */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class JoystickVelocityController : public rclcpp::Node
{
public:
    struct WhillDynamic
    {
        double max_linear_vel_{0};
        double min_linear_vel_{0};
        double max_yaw_rate_{0};
        double max_acceleration_{0};
        double max_deceleration_{0};
        double max_yaw_acceleration_{0};
    };

    JoystickVelocityController()
    :   Node("velocity_by_joystick_control_node")
    {
        this->declare_parameter("max_linear_vel", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("min_linear_vel", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("max_yaw_rate", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("max_acceleration", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("max_decceleration", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("max_yaw_acceleration", rclcpp::PARAMETER_DOUBLE);

        whill_dynamic_.max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        whill_dynamic_.min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
        whill_dynamic_.max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
        whill_dynamic_.max_yaw_acceleration_ = this->get_parameter("max_yaw_acceleration").as_double();
        whill_dynamic_.max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        whill_dynamic_.max_deceleration_ = this->get_parameter("max_decceleration").as_double();

        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/controller/joy", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/whill/states/joy", rclcpp::SensorDataQoS(),
                    std::bind(&JoystickVelocityController::subscriber_callback, this, std::placeholders::_1)); 
        timer_ = this->create_wall_timer(
        100ms, std::bind(&JoystickVelocityController::timer_callback, this));
    }

private:
    sensor_msgs::msg::Joy joy_;
    bool is_joystick_updated_{false};
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    WhillDynamic whill_dynamic_;
    void subscriber_callback(const sensor_msgs::msg::Joy &msg)
    {
        joy_ = msg;
        is_joystick_updated_ = true;
    
    }

    void timer_callback()
    {
        auto joy_vel{calculate_velocity_from_joy()};
        joy_pub_->publish(calculate_joy_from_velocity(joy_vel));
    }

    geometry_msgs::msg::Twist calculate_velocity_from_joy()
    {
        geometry_msgs::msg::Twist joy_vel{};

            //Linear velocity
            /*Forward movement*/
            if (joy_.axes[1] >= 0)
            {
                joy_vel.linear.x = joy_.axes[1] * whill_dynamic_.max_linear_vel_;
            }
            /*Backward movement*/
            else if (joy_.axes[1] < 0)
            {
                joy_vel.linear.x = -joy_.axes[1] * whill_dynamic_.min_linear_vel_;
            }

            //Angular velocity
            joy_vel.angular.z = joy_.axes[0] * whill_dynamic_.max_yaw_rate_;
        
        std::cout << "Linear vel     " <<  joy_vel.linear.x << std::endl;
        std::cout << "Angular vel     " <<  joy_vel.angular.z << std::endl;
        return joy_vel;
    }

    sensor_msgs::msg::Joy calculate_joy_from_velocity(geometry_msgs::msg::Twist  vel)
    {
        sensor_msgs::msg::Joy joy{};
        joy.axes.resize(2);
        /*Forward backward movement*/

        if (vel.linear.x >= 0 && vel.linear.x <= whill_dynamic_.max_linear_vel_)
        {
            joy.axes[1] = vel.linear.x / whill_dynamic_.max_linear_vel_ ;
        }
        else if (vel.linear.x < 0 && vel.linear.x >= whill_dynamic_.min_linear_vel_)
        {
            joy.axes[1] = - vel.linear.x/whill_dynamic_.min_linear_vel_ ;
        }

        if(vel.angular.z >= - whill_dynamic_.max_yaw_rate_ && vel.angular.z <= whill_dynamic_.max_yaw_rate_)
        {
            joy.axes[0] =vel.angular.z/whill_dynamic_.max_yaw_rate_;
        }
        return joy;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickVelocityController>());
    rclcpp::shutdown();
    return 0;
}