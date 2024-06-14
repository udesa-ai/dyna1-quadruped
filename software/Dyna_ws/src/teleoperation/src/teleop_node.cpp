#include "rclcpp/rclcpp.hpp"

#include <math.h>
#include <string>
#include <vector>

#include "teleop.hpp"
#include "std_msgs/msg/bool.hpp"
#include "teleop_msgs/msg/joy_buttons.hpp"
#include <chrono>

using std::placeholders::_1;

class TeleopNode : public rclcpp::Node
{
    public:
        TeleopNode() : Node("teleop_node")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"STARTING NODE: Teleoperation");
            double frequency = 60;
            int linear_x = 4;
            int linear_y = 3;
            int linear_z = 1;
            int angular = 0;
            int sw = 0;
            int es = 1;
            int RB = 5;
            int LB = 2;
            int RT = 5;
            int LT = 4;
            int UD = 7;
            int LR = 6;
            int start = 7;
            double l_scale = 1.0;
            double a_scale = 1.0;
            double B_scale = 1.0;
            debounce_thresh = 0.15;

            this->declare_parameter("frequency", frequency);
            this->declare_parameter("axis_linear_x", linear_x);
            this->declare_parameter("axis_linear_y", linear_y);
            this->declare_parameter("axis_linear_z", linear_z);
            this->declare_parameter("axis_angular", angular);
            this->declare_parameter("scale_linear", l_scale);
            this->declare_parameter("scale_angular", a_scale);
            this->declare_parameter("scale_bumper", B_scale);
            this->declare_parameter("button_switch", sw);
            this->declare_parameter("button_estop", es);
            this->declare_parameter("rb", RB);
            this->declare_parameter("lb", LB);
            this->declare_parameter("rt", RT);
            this->declare_parameter("lt", LT);
            this->declare_parameter("updown", UD);
            this->declare_parameter("leftright", LR);
            this->declare_parameter("debounce_thresh", debounce_thresh);
            this->declare_parameter("start_button", start);

            frequency = this->get_parameter("frequency").as_double();
            linear_x = this->get_parameter("axis_linear_x").as_int();
            linear_y = this->get_parameter("axis_linear_y").as_int();
            linear_z = this->get_parameter("axis_linear_z").as_int();
            angular = this->get_parameter("axis_angular").as_int();
            l_scale = this->get_parameter("scale_linear").as_double();
            a_scale = this->get_parameter("scale_angular").as_double();
            B_scale = this->get_parameter("scale_bumper").as_double();
            debounce_thresh = this->get_parameter("debounce_thresh").as_double();
            sw = this->get_parameter("button_switch").as_int();
            es = this->get_parameter("button_estop").as_int();
            RB = this->get_parameter("rb").as_int();
            LB = this->get_parameter("lb").as_int();
            RT = this->get_parameter("rt").as_int();
            LT = this->get_parameter("lt").as_int();
            UD = this->get_parameter("updown").as_int();
            LR = this->get_parameter("leftright").as_int();
            start = this->get_parameter("start_button").as_int();
            
            teleop = new Teleop(linear_x, linear_y, linear_z, angular,
                                   l_scale, a_scale, LB, RB, B_scale, LT,
                                   RT, UD, LR, sw, es, start);

            switched = true;
            switch_movement_pub = this->create_publisher<std_msgs::msg::Bool>("switch_movement_rec", 1);
            switch_movement_sub = this->create_subscription<std_msgs::msg::Bool>("switch_movement_rec_answ", 1,
                                       std::bind(&TeleopNode::switch_answer, this, _1));

            estop_pub = this->create_publisher<std_msgs::msg::Bool>("estop", 1);

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("teleop", 1);
            
            jb_pub = this->create_publisher<teleop_msgs::msg::JoyButtons>("joybuttons", 1);

            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
                                       std::bind(&TeleopNode::joyCallback, this, _1));

            timer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000/frequency)),
                std::bind(&TeleopNode::process, this)
            );

            current_time = this->get_clock()->now();
            last_time = this->get_clock()->now();
        }
    
    private:
        void switch_answer(const std_msgs::msg::Bool::SharedPtr msg) 
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"switching response received");
            (void)msg;
            switched = true;
        }
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) 
        {
            teleop->joyCallback(joy);
        }
        void process()
        {
            current_time = this->get_clock()->now();
            std_msgs::msg::Bool estop;
            estop.data = teleop->return_estop();
            bool switch_trigger = teleop->return_trigger();

            if (estop.data and current_time.seconds() - last_time.seconds() >= debounce_thresh)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"SENDING ESTOP COMMAND");
                last_time = this->get_clock()->now();
                
            } else if (!switch_trigger and switched)
            {
                vel_pub->publish(teleop->return_twist());
                estop.data = 0;
            } else if (switch_trigger and current_time.seconds() - last_time.seconds() >= debounce_thresh)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending switching command");
                switched = false;
                estop.data = 0;
                std_msgs::msg::Bool switchpls;
                switchpls.data = true;
                switch_movement_pub->publish(switchpls);
                last_time = this->get_clock()->now();
            }

            jb_pub->publish(teleop->return_buttons());

            estop_pub->publish(estop);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        bool switched;
        rclcpp::Time current_time;
        rclcpp::Time last_time;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_movement_pub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_movement_sub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Publisher<teleop_msgs::msg::JoyButtons>::SharedPtr jb_pub;
        Teleop* teleop;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        double debounce_thresh;

        

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}