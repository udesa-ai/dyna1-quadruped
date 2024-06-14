# include "ros2socketcan.h"
# include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


ros2socketcan::ros2socketcan(std::string can_socket2): Node("ros2" + can_socket2), stream(ios), signals(ios, SIGINT, SIGTERM)
{

}

void ros2socketcan::Init(const char* can_socket)
{
    printf("Using can socket %s\n", can_socket);
    
    const char* canname = can_socket;
        
    topicname_receive 	<< "CAN/" << canname << "/" << "receive";
    topicname_transmit 	<< "CAN/" << canname << "/" << "transmit";
      
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::QoS rmw_qos_profile_sensor_data(24);
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = client_cb_group_;
    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;
    publisher_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(),rmw_qos_profile_sensor_data);
    subscription_ 	= this->create_subscription<can_msgs::msg::Frame>(topicname_transmit.str(), rmw_qos_profile_sensor_data, std::bind(&ros2socketcan::CanPublisher, this, _1), options);
    
    strcpy(ifr.ifr_name, can_socket);
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }

    stream.assign(natsock);
    
    std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener, this,std::ref(rec_frame),std::ref(stream)));
    
    signals.async_wait(std::bind(&ros2socketcan::stop, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
    
    rclcpp::spin(shared_from_this());

}

void ros2socketcan::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

ros2socketcan::~ros2socketcan(){printf("\nEnd of Publisher Thread. \n");}

void ros2socketcan::CanSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;
    
    frame1.can_id = msg.id;
    
    if (msg.eff == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
    }
    
    if (msg.err == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
    }
    
    if (msg.rtr == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
    }
    
    frame1.can_dlc = msg.dlc;

    for(int i=0;i<(int)frame1.can_dlc;i++)
    {
        frame1.data[i] = msg.data[i];
    }
     
    // std::cout << "S | " << frame1.can_id << " | ";

    // for (int j = static_cast<int>(frame1.can_dlc)-1; j > -1 ; j--) {
    //     std::cout << static_cast<u_int16_t>(frame1.data[j]) << " ";
    // }

    // std::cout << std::endl;

    // printf("S | %x | %s | ", frame1.can_id, frame1.data);
    // for(int j=0;j<(int)frame1.can_dlc;j++)
    // {
    //     printf("%i ", frame1.data[j]);
    // }
    // printf("\n");
    
    stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&ros2socketcan::CanSendConfirm, this));
}


void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{
    can_msgs::msg::Frame msg1;
    msg1.id  = msg->id;
    msg1.dlc = msg->dlc;
    msg1.eff = msg->eff;
    msg1.rtr = msg->rtr;
    msg1.err = msg->err;
    msg1.data= msg->data;
    
    CanSend(msg1);
    
}

void ros2socketcan::CanSendConfirm(void)
{
    //std::cout << "Message sent" << std::endl;
}

void ros2socketcan::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    
    can_msgs::msg::Frame frame;
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
        frame.data[i]=rec_frame.data[i];
    }
    

    publisher_->publish(frame);
    
    // if (rec_frame.can_id == 20)
    // {
    //     std::cout << "R | " << rec_frame.can_id << " | ";

    //     for (int j = static_cast<int>(rec_frame.can_dlc)-1; j > -1 ; j--) {
    //         std::cout << static_cast<u_int16_t>(rec_frame.data[j]) << " ";
    //     }

    //     std::cout << std::endl;
    // }
    
    

    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}

int main(int argc, char *argv[])
{
    std::cout << programdescr << std::endl;
    rclcpp::init(argc, argv);
    
    auto ros2canptr = std::make_shared<ros2socketcan>();
    ros2canptr -> Init();
    
    
    return 0;
}
