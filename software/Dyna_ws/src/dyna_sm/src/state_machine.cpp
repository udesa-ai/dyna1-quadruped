#include "state_machine.hpp"


using std::placeholders::_1;

StateMachine::StateMachine(): Node("state_machine")
{   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"STARTING NODE: spot_mini State Machine");

    double frequency = 5;
    timeout = 1.0;

    alpha = 0.1;

    this->declare_parameter("frequency", frequency);
    frequency = this->get_parameter("frequency").as_double();

    teleop_flag = false;
    motion_flag = false;
    ESTOP = true;
    estop_pressed = false;

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    cmd.motion = Stop;
    cmd.movement = Viewing;
    cmd.x_velocity = 0.0;
    cmd.y_velocity = 0.0;
    cmd.rate = 0.0;
    cmd.roll = 0.0;
    cmd.pitch = 0.0;
    cmd.yaw = 0.0;
    cmd.z = 0.0;
    cmd.faster = 0.0;
    cmd.slower = 0.0;

    mini_cmd.x_velocity = 0.0;
    mini_cmd.y_velocity = 0.0;
    mini_cmd.rate = 0.0;
    mini_cmd.roll = 0.0;
    mini_cmd.pitch = 0.0;
    mini_cmd.yaw = 0.0;
    mini_cmd.z = 0.0;
    mini_cmd.faster = 0.0;
    mini_cmd.slower = 0.0;
    mini_cmd.motion = "Stop";
    mini_cmd.movement = "Stepping";

    switch_movement_pub = this->create_publisher<std_msgs::msg::Bool>("switch_movement_rec_answ", 1);
    switch_movement_sub = this->create_subscription<std_msgs::msg::Bool>("switch_movement_rec", 1,
                            std::bind(&StateMachine::swm_callback, this, _1));

    teleop_sub = this->create_subscription<geometry_msgs::msg::Twist>("teleop", 1,
                            std::bind(&StateMachine::teleop_callback, this, _1));
    estop_sub = this->create_subscription<std_msgs::msg::Bool>("estop", 1,
                            std::bind(&StateMachine::estop_callback, this, _1));

    mini_pub = this->create_publisher<joint_msgs::msg::MiniCmd>("mini_cmd", 1);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/frequency)),
        std::bind(&StateMachine::process, this)
    );

}

void StateMachine::process()
{
    current_time = this->get_clock()->now();

    if (!motion_flag and !(current_time.seconds() - last_time.seconds() > timeout) and !ESTOP)
    {
        mini_cmd.x_velocity = cmd.x_velocity;
        mini_cmd.y_velocity = cmd.y_velocity;
        mini_cmd.rate = cmd.rate;
        mini_cmd.roll = cmd.roll;
        mini_cmd.pitch = cmd.pitch;
        mini_cmd.yaw = cmd.yaw;
        mini_cmd.z = cmd.z;
        mini_cmd.faster = cmd.faster;
        mini_cmd.slower = cmd.slower;

        if (cmd.motion == Go)
        {
            mini_cmd.motion = "Go";
        } else
        {
            mini_cmd.motion = "Stop";
        }
        // Movement
        if (cmd.movement == Stepping)
        {
            mini_cmd.movement = "Stepping";
        } else
        {
            mini_cmd.movement = "Viewing";
        }

    } else
    {
        mini_cmd.x_velocity = 0.0;
        mini_cmd.y_velocity = 0.0;
        mini_cmd.rate = 0.0;
        mini_cmd.roll = 0.0;
        mini_cmd.pitch = 0.0;
        mini_cmd.yaw = 0.0;
        mini_cmd.z = 0.0;
        mini_cmd.faster = 0.0;
        mini_cmd.slower = 0.0;
        mini_cmd.motion = "Stop";
    }

    if (!ESTOP and current_time.seconds() - last_time.seconds() > timeout)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"TIMEOUT...ENGAGING E-STOP!");
        ESTOP = true;
        this->update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    mini_pub->publish(mini_cmd);
    motion_flag = false;
}

constexpr bool StateMachine::almost_equal(double d1, double d2, double epsilon)
{
    if (fabs(d1 - d2) < epsilon)
    {
        return true;
    } else {
        return false;
    }
}

void StateMachine::teleop_callback(const geometry_msgs::msg::Twist::SharedPtr tw)
{ 
  this->update_command(tw->linear.x, tw->linear.y, tw->linear.z, tw->angular.z, tw->angular.x, tw->angular.y);
}

void StateMachine::estop_callback(const std_msgs::msg::Bool::SharedPtr estop)
{ 
    if (estop->data and !estop_pressed)
    {
        estop_pressed = true;
        this->update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        motion_flag = true;
        if (!ESTOP)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ENGAGING MANUAL E-STOP!");
            ESTOP = true;
        } else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"DIS-ENGAGING MANUAL E-STOP!");
            ESTOP = false;
        }
    } else if (!estop->data){
        estop_pressed = false;
    }

    last_time = this->get_clock()->now();
}


void StateMachine::swm_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    (void)msg;
    this->switch_movement();
    motion_flag = true;
    std_msgs::msg::Bool msg2;
    msg2.data = true;
    switch_movement_pub->publish(msg2);
}

void StateMachine::update_command(const double & vx, const double & vy, const double & z,
							  const double & w, const double & wx, const double & wy)
{
    if (almost_equal(vx, 0.0) and almost_equal(vy, 0.0) and almost_equal(z, 0.0) and almost_equal(w, 0.0))
    {
        cmd.motion = Stop;
        cmd.x_velocity = 0.0;
        cmd.y_velocity = 0.0;
        cmd.rate = 0.0;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;
        cmd.z = 0.0;
        cmd.faster = 0.0;
        cmd.slower = 0.0;
    } else
    {
        cmd.motion = Go;
        if (cmd.movement == Stepping)
        {
            // Stepping Mode, use commands as vx, vy, rate, Z
            cmd.x_velocity = filter(vx, cmd.x_velocity);
            cmd.y_velocity = filter(vy, cmd.y_velocity);
            cmd.rate = filter(w, cmd.rate);
            cmd.z = filter(z, cmd.z);
            cmd.roll = 0.0;
            cmd.pitch = 0.0;
            cmd.yaw = 0.0;
            // change clearance height from +- 0-2 * scaling
            cmd.faster = 1.0 - wx;
            cmd.slower = -(1.0 - wy);
        } else
        {
            // Viewing Mode, use commands as RPY, Z
            cmd.x_velocity = 0.0;
            cmd.y_velocity = 0.0;
            cmd.rate = 0.0;
            cmd.roll = filter(vy, cmd.roll);
            cmd.pitch = filter(vx, cmd.pitch);
            cmd.yaw = filter(w, cmd.yaw);
            cmd.z = filter(z, cmd.z);
            cmd.faster = 0.0;
            cmd.slower = 0.0;
        }
    }
}

void StateMachine::switch_movement()
{
    if (!almost_equal(cmd.x_velocity, 0.0) and !almost_equal(cmd.y_velocity, 0.0) and !almost_equal(cmd.rate, 0.0))
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"MAKE SURE BOTH LINEAR [%.2f, %.2f] AND ANGULAR VELOCITY [%.2f] ARE AT 0.0 BEFORE SWITCHING!", cmd.x_velocity, cmd.y_velocity, cmd.rate);

        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"STOPPING ROBOT...");

        cmd.motion = Stop;
        cmd.x_velocity = 0.0;
        cmd.y_velocity = 0.0;
        cmd.rate = 0.0;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;
        cmd.z = 0.0;
        cmd.faster = 0.0;
        cmd.slower = 0.0;
    } else
    {
        cmd.x_velocity = 0.0;
        cmd.y_velocity = 0.0;
        cmd.rate = 0.0;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;
        cmd.z = 0.0;
        cmd.faster = 0.0;
        cmd.slower = 0.0;
        if (cmd.movement == Viewing)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"SWITCHING TO STEPPING MOTION, COMMANDS NOW MAPPED TO VX|VY|W|Z.");

            cmd.movement = Stepping;
            cmd.motion = Stop;
        } else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"SWITCHING TO VIEWING MOTION, COMMANDS NOW MAPPED TO R|P|Y|Z.");

            cmd.movement = Viewing;
            cmd.motion = Stop;
        }
    }
}

double StateMachine::filter(double value, double previous)
{
    return alpha * value + (1-alpha)*previous;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachine>());
  rclcpp::shutdown();
  return 0;
}