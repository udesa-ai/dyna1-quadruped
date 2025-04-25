#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "joint_msgs/msg/joints.hpp"
#include "joint_msgs/msg/joints_bool.hpp"
#include <std_msgs/msg/float32.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <memory>
#include <queue>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cstring>  // for memcpy
#include <cstdint>

using namespace std::placeholders;
using boost::asio::serial_port_base;
namespace asio = boost::asio;

class UARTBridgeNode : public rclcpp::Node {
public:
    UARTBridgeNode()
        : Node("uart_bridge_node"),
          io_context_(),
          serial_port_(io_context_),
          read_buffer_{} {

        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        publisher_motor_positions_ = this->create_publisher<joint_msgs::msg::Joints>("motor_positions", 10);
        publisher_motor_velocities_ = this->create_publisher<joint_msgs::msg::Joints>("motor_velocities", 10);
        publisher_motor_currents_ = this->create_publisher<joint_msgs::msg::Joints>("motor_currents", 10);


        subscriber_joints = this->create_subscription<joint_msgs::msg::Joints>(
            "request_positions", 10,
            std::bind(&UARTBridgeNode::request_positions, this, _1)
        );

        subscriber_max_current = this->create_subscription<std_msgs::msg::Float32>(
            "request_maxc", 10,
            std::bind(&UARTBridgeNode::request_max_current, this, _1)
        );

        subscriber_reboot = this->create_subscription<joint_msgs::msg::JointsBool>(
            "request_reboot", 10,
            std::bind(&UARTBridgeNode::request_reboot, this, _1)
        );
        
        subscriber_motor_state = this->create_subscription<joint_msgs::msg::JointsBool>(
            "request_axisstate", 10,
            std::bind(&UARTBridgeNode::request_motor_state, this, _1)
        );



        std::string port_name = "/dev/ttyTHS1";
        int baud_rate = 115200;

        boost::system::error_code ec;
        serial_port_.open(port_name, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", ec.message().c_str());
            return;
        }

        serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
        serial_port_.set_option(serial_port_base::character_size(8));
        serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        do_async_read();

        // Run boost::asio in a background thread
        io_thread_ = std::thread([this]() { io_context_.run(); });
    }

    ~UARTBridgeNode() {
        serial_port_.close();
        io_context_.stop();
        if (io_thread_.joinable())
            io_thread_.join();
    }

private:
    void do_async_read() {
        serial_port_.async_read_some(
            asio::buffer(read_buffer_, 1),
            std::bind(&UARTBridgeNode::handle_read, this, _1, _2)
        );
    }

    void handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!error && bytes_transferred > 0) {
            input_buffer_.push_back(read_buffer_[0]);

            // Try to find full frame
            while (input_buffer_.size() >= 9) {  // Minimum frame size
                // Look for sync header
                auto it = std::search(input_buffer_.begin(), input_buffer_.end(), sync_.begin(), sync_.end());
                if (it == input_buffer_.end()) {
                    input_buffer_.clear();
                    break;
                }

                std::stringstream ss;
                for (unsigned char c : input_buffer_) {
                    ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
                }

                size_t start = std::distance(input_buffer_.begin(), it);
                if (input_buffer_.size() < start + 9) {
                    break; // Minimum full frame = 3 sync + 1 topic + 1 seq + 2 len + 2 CRC
                }
                
                uint8_t topic_id = input_buffer_[start + 3];
                uint8_t len_high = input_buffer_[start + 5];
                uint8_t len_low  = input_buffer_[start + 6];
                uint16_t payload_len = (len_high << 8) | len_low;
                size_t frame_len = 3 + 1 + 1 + 2 + 2 + payload_len;  // sync + topic + seq + len + crc + payload

                if (input_buffer_.size() < start + frame_len){
                    break; // Wait for more data
                }
               

                std::vector<uint8_t> frame(input_buffer_.begin() + start, input_buffer_.begin() + start + frame_len);
                input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + start + frame_len);

                // CRC check
                uint16_t received_crc = (frame[7] << 8) | frame[8];
                uint16_t computed_crc = calculate_crc(frame.data() + 9, frame_len - 9);  // Exclude sync
                if (received_crc != computed_crc) {
                    RCLCPP_WARN(this->get_logger(), "CRC mismatch");
                    continue;
                }

                // Extract and publish payload
                const uint8_t* payload_ptr = reinterpret_cast<const uint8_t*>(&frame[9]);
                size_t payload_length = frame.size() - 9;
                handle_payload(topic_id, payload_ptr, payload_length);
            }
    
            do_async_read();  // continue reading
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", error.message().c_str());
        }
    }
    

    void handle_payload(uint8_t topic_id, const uint8_t* payload, size_t length){
        switch (topic_id) {
            case 1: {
                // Topic 1: accel (xyz 4 bytes each), gyro (xyz 4 bytes each), pos (12 motors 4 bytes each), vel (12 motors 4 bytes each)
                if (length != 168) {
                    std::cout << "Invalid payload length for topic 1: " << length << std::endl;
                    return;
                }
                std::vector<float> floats;
                for (size_t i = 0; i < length; i += 4) {
                    uint32_t temp;
                    std::memcpy(&temp, payload + i, sizeof(uint32_t));  // Copy bytes into a 32-bit temp
                    temp = swap_bytes(temp);  // Swap bytes if necessary (if endian mismatch)
                    
                    float value;
                    std::memcpy(&value, &temp, sizeof(float));  // Convert the 32-bit temp to float
                    
                    floats.push_back(value);
                }

                sensor_msgs::msg::Imu msg;
                msg.linear_acceleration.x = floats[0];
                msg.linear_acceleration.y = floats[1];
                msg.linear_acceleration.z = floats[2];
                msg.angular_velocity.x = floats[3];
                msg.angular_velocity.y = floats[4];
                msg.angular_velocity.z = floats[5];
                publisher_imu_->publish(msg);

                // RCLCPP_INFO(this->get_logger(), "Published IMU data: [%.2f, %.2f, %.2f]", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

                joint_msgs::msg::Joints motor_positions;
                joint_msgs::msg::Joints motor_velocities;
                joint_msgs::msg::Joints motor_currents;
                
                uint8_t pos_start = 6;
                uint8_t vel_start = 18;
                uint8_t curr_start = 30;

                motor_positions.frshoulder = floats[0 + pos_start];
                motor_velocities.frshoulder = floats[0 + vel_start];
                motor_currents.frshoulder = floats[0 + curr_start];

                motor_positions.frarm = floats[1 + pos_start];
                motor_velocities.frarm = floats[1 + vel_start];
                motor_currents.frarm = floats[1 + curr_start];

                motor_positions.frfoot = floats[2 + pos_start];
                motor_velocities.frfoot = floats[2 + vel_start];
                motor_currents.frfoot = floats[2 + curr_start];                

                motor_positions.flshoulder = floats[3 + pos_start];
                motor_velocities.flshoulder = floats[3 + vel_start];
                motor_currents.flshoulder = floats[3 + curr_start];
                
                motor_positions.flarm = floats[4 + pos_start];
                motor_velocities.flarm = floats[4 + vel_start];
                motor_currents.flarm = floats[4 + curr_start];

                motor_positions.flfoot = floats[5 + pos_start];
                motor_velocities.flfoot = floats[5 + vel_start];
                motor_currents.flfoot = floats[5 + curr_start];
                
                motor_positions.blshoulder = floats[6 + pos_start];
                motor_velocities.blshoulder = floats[6 + vel_start];
                motor_currents.blshoulder = floats[6 + curr_start];

                motor_positions.blarm = floats[7 + pos_start];
                motor_velocities.blarm = floats[7 + vel_start];
                motor_currents.blarm = floats[7 + curr_start];

                motor_positions.blfoot = floats[8 + pos_start];
                motor_velocities.blfoot = floats[8 + vel_start];
                motor_currents.blfoot = floats[8 + curr_start];

                motor_positions.brshoulder = floats[9 + pos_start];
                motor_velocities.brshoulder = floats[9 + vel_start];
                motor_currents.brshoulder = floats[9 + curr_start];

                motor_positions.brarm = floats[10 + pos_start];
                motor_velocities.brarm = floats[10 + vel_start];
                motor_currents.brarm = floats[10 + curr_start];

                motor_positions.brfoot = floats[11 + pos_start];
                motor_velocities.brfoot = floats[11 + vel_start];
                motor_currents.brfoot = floats[11 + curr_start];

                publisher_motor_positions_->publish(motor_positions);
                publisher_motor_velocities_->publish(motor_velocities);
                publisher_motor_currents_->publish(motor_currents);
                // RCLCPP_INFO(this->get_logger(), "Published motor positions, velocities, and currents");                

                break;
            }

            case 4: { // debug
                if (length != 48) {
                    std::cout << "Invalid payload length for topic 1: " << length << std::endl;
                    return;
                }
                std::vector<float> floats;
                for (size_t i = 0; i < length; i += 4) {
                    uint32_t temp;
                    std::memcpy(&temp, payload + i, sizeof(uint32_t));  // Copy bytes into a 32-bit temp
                    //temp = swap_bytes(temp);  // Swap bytes if necessary (if endian mismatch)
                    
                    float value;
                    std::memcpy(&value, &temp, sizeof(float));  // Convert the 32-bit temp to float
                    
                    floats.push_back(value);
                }

                joint_msgs::msg::Joints motor_positions;
                
                uint8_t pos_start = 0;

                motor_positions.frshoulder = floats[0 + pos_start];

                motor_positions.frarm = floats[1 + pos_start];

                motor_positions.frfoot = floats[2 + pos_start];

                motor_positions.flshoulder = floats[3 + pos_start];
                
                motor_positions.flarm = floats[4 + pos_start];
                
                motor_positions.flfoot = floats[5 + pos_start];
               
                motor_positions.blshoulder = floats[6 + pos_start];
                
                motor_positions.blarm = floats[7 + pos_start];
    
                motor_positions.blfoot = floats[8 + pos_start];
                
                motor_positions.brshoulder = floats[9 + pos_start];
                
                motor_positions.brarm = floats[10 + pos_start];
                
                motor_positions.brfoot = floats[11 + pos_start];
                
                publisher_motor_positions_->publish(motor_positions);
                // RCLCPP_INFO(this->get_logger(), "Published motor positions and velocities");                

                break;
            }

            // case 2: {
            //     std_msgs::msg::String msg;
            //     msg.data = payload;
            //     publisher_->publish(msg);
            //     RCLCPP_INFO(this->get_logger(), "Published payload: %s", msg.data.c_str());
            // }
            
            default:
                std::cout << "Unknown topic ID: " << static_cast<int>(topic_id) << std::endl;
                break;
        }
    }

    uint32_t swap_bytes(uint32_t value) {
        return ((value >> 24) & 0x000000FF) |
               ((value << 8)  & 0x00FF0000) |
               ((value >> 8)  & 0x0000FF00) |
               ((value << 24) & 0xFF000000);
    }

    void request_positions(const joint_msgs::msg::Joints::SharedPtr msg) {
        uint8_t topic_id = 0x04;
        std::vector<float> floats;
        floats.push_back(msg->frshoulder);
        floats.push_back(msg->frarm);
        floats.push_back(msg->frfoot);
        floats.push_back(msg->flshoulder);
        floats.push_back(msg->flarm);
        floats.push_back(msg->flfoot);
        floats.push_back(msg->blshoulder);
        floats.push_back(msg->blarm);
        floats.push_back(msg->blfoot);
        floats.push_back(msg->brshoulder);
        floats.push_back(msg->brarm);
        floats.push_back(msg->brfoot);
        
        // Vector to hold the payload of bytes
        std::vector<uint8_t> payload;
        payload.reserve(floats.size() * sizeof(float)); // Reserve 48 bytes for efficiency

        // Convert each float to 4 bytes
        for (const float& f : floats) {
            uint8_t bytes[sizeof(float)];
            std::memcpy(bytes, &f, sizeof(float));  // Copy float into byte array

            // Append bytes to the payload vector
            payload.insert(payload.end(), bytes, bytes + sizeof(float));
        }

        uart_write_callback(payload, topic_id);  // Send the payload
    }


    void request_max_current(const std_msgs::msg::Float32::SharedPtr msg) {
        uint8_t topic_id = 0x05;
        float max_current = msg->data;
        
        // Vector to hold the payload of bytes
        std::vector<uint8_t> payload;
        payload.reserve(sizeof(float)); // Reserve 4 bytes for efficiency

        // Convert each float to 4 bytes
        uint8_t bytes[sizeof(float)];
        std::memcpy(bytes, &max_current, sizeof(float));  // Copy float into byte array

        // Append bytes to the payload vector
        payload.insert(payload.end(), bytes, bytes + sizeof(float));

        uart_write_callback(payload, topic_id);  // Send the payload
    }

    void request_reboot(const joint_msgs::msg::JointsBool::SharedPtr msg){
        uint8_t topic_id = 0x06;
        send_bits(msg, topic_id);
        // RCLCPP_INFO(this->get_logger(), "received reboot request");
    }

    void request_motor_state(const joint_msgs::msg::JointsBool::SharedPtr msg) {
        uint8_t topic_id = 0x07;
        send_bits(msg, topic_id);
    }

    void send_bits(const joint_msgs::msg::JointsBool::SharedPtr msg, uint8_t topic_id) {
        uint16_t bitfield = 0x0000;
        bitfield |= ((msg->frshoulder) << 0);
        bitfield |= ((msg->frarm) << 1);
        bitfield |= ((msg->frfoot) << 2);
        bitfield |= ((msg->flshoulder) << 3);
        bitfield |= ((msg->flarm) << 4);
        bitfield |= ((msg->flfoot) << 5);
        bitfield |= ((msg->blshoulder) << 6);
        bitfield |= ((msg->blarm) << 7);
        bitfield |= ((msg->blfoot) << 8);
        bitfield |= ((msg->brshoulder) << 9);
        bitfield |= ((msg->brarm) << 10);
        bitfield |= ((msg->brfoot) << 11);

        // Vector to hold the payload of bytes
        std::vector<uint8_t> payload;
        payload.reserve(sizeof(uint16_t)); // Reserve 4 bytes for efficiency

        payload.push_back(static_cast<uint8_t>((bitfield >> 8) & 0xFF)); // upper byte
        payload.push_back(static_cast<uint8_t>(bitfield & 0xFF));        // lower byte
        
        uart_write_callback(payload, topic_id);  // Send the payload
    }



    // Writing
    void uart_write_callback(const std::vector<uint8_t>& payload, uint8_t topic_id) {
        uint16_t len = payload.size();

        // Frame format
        std::vector<uint8_t> frame;
        frame.push_back('>');
        frame.push_back('>');
        frame.push_back('>');

        frame.push_back(topic_id);

        frame.push_back(seq_counter_++);  // sequence number

        // Length
        frame.push_back((len >> 8) & 0xFF);  // len_high
        frame.push_back(len & 0xFF);        // len_low

        // Placeholder for CRC
        frame.push_back(0x00);  // CRC high
        frame.push_back(0x00);  // CRC low

        // Append payload
        frame.insert(frame.end(), payload.begin(), payload.end());

        // Compute CRC over topic_id to end of payload (not sync)
        uint16_t crc = calculate_crc(frame.data() + 9, frame.size() - 9);
        frame[7] = (crc >> 8) & 0xFF;
        frame[8] = crc & 0xFF;

        // Convert to string for sending
        std::string frame_str(frame.begin(), frame.end());


        bool write_in_progress = !this->write_queue_.empty();
        this->write_queue_.push(frame_str);
        if (!write_in_progress) {
            do_async_write();
        }
    }


    void do_async_write() {
        if (this->write_queue_.empty()) return;

        const std::string &data = this->write_queue_.front();
        asio::async_write(
            serial_port_,
            asio::buffer(data),
            [this](const boost::system::error_code &ec, std::size_t /*bytes_transferred*/) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", ec.message().c_str());
                } else {
                    std::stringstream ss;
                    const std::string& data = this->write_queue_.front();

                    for (unsigned char c : data) {
                    ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
                    }

                    // RCLCPP_INFO(this->get_logger(), "Wrote to serial: %s", ss.str().c_str());
                    this->write_queue_.pop();
                    if (!this->write_queue_.empty()) {
                        do_async_write();  // continue writing queued data
                    }
                }
            }
        );
    }

    uint16_t calculate_crc(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr publisher_motor_positions_;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr publisher_motor_velocities_;
    rclcpp::Publisher<joint_msgs::msg::Joints>::SharedPtr publisher_motor_currents_;

    rclcpp::Subscription<joint_msgs::msg::Joints>::SharedPtr subscriber_joints;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_max_current;
    rclcpp::Subscription<joint_msgs::msg::JointsBool>::SharedPtr subscriber_reboot;
    rclcpp::Subscription<joint_msgs::msg::JointsBool>::SharedPtr subscriber_motor_state;

    asio::io_context io_context_;
    asio::serial_port serial_port_;
    std::thread io_thread_;

    char read_buffer_[1];
    std::queue<std::string> write_queue_;
    uint8_t seq_counter_ = 0;
    const std::array<uint8_t, 3> sync_ = {'>', '>', '>'};
    std::vector<uint8_t> input_buffer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UARTBridgeNode>());
  rclcpp::shutdown();
  return 0;
}