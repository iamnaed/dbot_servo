#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

class DbotServoKeyboardPublisher : public rclcpp::Node
{
public:
    DbotServoKeyboardPublisher(std::string node_name, rclcpp::NodeOptions options, std::string joint_jog_topic, std::string cartesian_jog_topic) : Node(node_name, options)
    {
        joint_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_jog_topic, 10);
        cartesian_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_jog_topic, 10);
        timer_publish_ = this->create_wall_timer(50ms, std::bind(&DbotServoKeyboardPublisher::publish_loop, this));
        
        // Set
        frame_to_publish_ = EEF_FRAME_ID;

        // Value
        joint_vel_cmd_ = 1.0;
        joint_jog_msg_.joint_names.resize(6);
        joint_jog_msg_.joint_names = {"j0_joint", "j1_joint", "j2_joint", "j3_joint", "j4_joint", "j5_joint"};
        joint_jog_msg_.velocities.resize(6);
        joint_jog_msg_.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Flags
        is_publish_joint_.store(false);
        is_publish_cartesian_.store(false);
        
        // Thread
        is_keyboard_reading_.store(true);
        keyboard_thread_ = std::thread{&DbotServoKeyboardPublisher::keyboard_loop, this};
    }

    ~DbotServoKeyboardPublisher()
    {        
        // Thread
        is_keyboard_reading_.store(false);
        RCLCPP_INFO(this->get_logger(), "Waiting for keyboard_loop thread to finish, press any key to continue stopping");
        keyboard_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Servo ended. . .");
    }

private:
    void publish_loop()
    {
        // auto msg = get_cartesian_jog_msg();
        // cartesian_cmd_pub_->publish(msg);
        // auto msg2 = get_joint_jog_msg();
        // joint_cmd_pub_->publish(msg2);

        // Message
        if (is_publish_cartesian_.load())
        {
            // Publish
            auto msg = get_cartesian_jog_msg();
            cartesian_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing cartesian jogging commands. . .");

            // Reset
            is_publish_cartesian_.store(false);
        }
        else if (is_publish_joint_.load())
        {
            // Publish
            auto msg = get_joint_jog_msg();
            joint_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing joint jogging commands. . .");

            // Reset
            is_publish_joint_.store(false);
        }
    }

    void keyboard_loop()
    {
        char c;
        //std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
        puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
        puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
        puts("'Q' to quit.");

        geometry_msgs::msg::Vector3 lin;

        while(true)
        {
            // get the next event from the keyboard
            try
            {
                input_.readOne(&c);
            }
            catch (const std::runtime_error&)
            {
                perror("read():");
                return;
            }

            // Guard
            // Placed here so that no command is executed if it is already false
            if(!is_keyboard_reading_.load())
                break;

            // Use read key-press
            switch (c)
            {
            case KEYCODE_LEFT:
                RCLCPP_INFO(this->get_logger(), "LEFT");
                lin.x = 0.0;
                lin.y = -1.0;
                lin.z = 0.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_RIGHT:
                RCLCPP_INFO(this->get_logger(), "RIGHT");
                lin.x = 0.0;
                lin.y = 1.0;
                lin.z = 0.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_UP:
                RCLCPP_INFO(this->get_logger(), "UP");
                lin.x = 1.0;
                lin.y = 0.0;
                lin.z = 0.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_DOWN:
                RCLCPP_INFO(this->get_logger(), "DOWN");
                lin.x = -1.0;
                lin.y = 0.0;
                lin.z = 0.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_PERIOD:
                RCLCPP_INFO(this->get_logger(), "PERIOD");
                lin.x = 0.0;
                lin.y = 0.0;
                lin.z = -1.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_SEMICOLON:
                RCLCPP_INFO(this->get_logger(), "SEMI-COLON");
                lin.x = 0.0;
                lin.y = 0.0;
                lin.z = 1.0;
                set_cartesian_jog_msg(lin);
                is_publish_cartesian_.store(true);
                break;
            case KEYCODE_E:
                RCLCPP_INFO(this->get_logger(), "E");
                frame_to_publish_ = EEF_FRAME_ID;
                break;
            case KEYCODE_W:
                RCLCPP_INFO(this->get_logger(), "W");
                frame_to_publish_ = BASE_FRAME_ID;
                break;
            case KEYCODE_1:
                RCLCPP_INFO(this->get_logger(), "1");
                set_joint_jog_msg(0, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_2:
                RCLCPP_INFO(this->get_logger(), "2");
                set_joint_jog_msg(1, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_3:
                RCLCPP_INFO(this->get_logger(), "3");
                set_joint_jog_msg(2, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_4:
                RCLCPP_INFO(this->get_logger(), "4");
                set_joint_jog_msg(3, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_5:
                RCLCPP_INFO(this->get_logger(), "5");
                set_joint_jog_msg(4, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_6:
                RCLCPP_INFO(this->get_logger(), "6");
                set_joint_jog_msg(5, joint_vel_cmd_);
                is_publish_joint_.store(true);
                break;
            case KEYCODE_7:
                RCLCPP_INFO(this->get_logger(), "7");
                //is_publish_joint_.store(true);
                break;
            case KEYCODE_R:
                RCLCPP_INFO(this->get_logger(), "R");
                joint_vel_cmd_ *= -1;
                break;
            case KEYCODE_Q:
                RCLCPP_INFO(this->get_logger(), "quit");
                is_keyboard_reading_.store(false);
                break;
            } // switch
        } // while

        input_.shutdown();
        puts("Keyboard loop ended");
    }

    void set_cartesian_jog_msg(const geometry_msgs::msg::Vector3& linear)
    {
        const std::lock_guard<std::mutex> lock(mtx_cartesian_);

        // Set
        cartesian_jog_msg_.header.stamp = this->now();
        cartesian_jog_msg_.header.frame_id = frame_to_publish_;
        cartesian_jog_msg_.twist.linear = linear;
    }

    geometry_msgs::msg::TwistStamped get_cartesian_jog_msg()
    {
        const std::lock_guard<std::mutex> lock(mtx_cartesian_);
        return cartesian_jog_msg_;
    }

    void set_joint_jog_msg(int idx, double vel)
    {
        const std::lock_guard<std::mutex> lock(mtx_joint_);

        // Set
        joint_jog_msg_.header.stamp = this->now();
        joint_jog_msg_.header.frame_id = BASE_FRAME_ID;

        for (size_t i = 0; i < joint_jog_msg_.velocities.size(); i++)
        {
            int ii = (int)i;
            joint_jog_msg_.velocities[ii] = (ii==idx) ? vel : 0;
        }
        
    }

    control_msgs::msg::JointJog get_joint_jog_msg()
    {
        const std::lock_guard<std::mutex> lock(mtx_joint_);
        return joint_jog_msg_;
    }

private:

    // Constants
    const std::string EEF_FRAME_ID = "tcp_link";
    const std::string BASE_FRAME_ID = "base_link";

    // Publishers
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_cmd_pub_;

    // Threading
    std::mutex mtx_joint_;
    std::mutex mtx_cartesian_;
    std::thread keyboard_thread_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_publish_;

    // Robot
    std::vector<std::string> joint_names_;
    std::string frame_to_publish_;

    // Messages
    control_msgs::msg::JointJog joint_jog_msg_;
    geometry_msgs::msg::TwistStamped cartesian_jog_msg_;

    // Flags
    std::atomic<bool> is_publish_joint_;
    std::atomic<bool> is_publish_cartesian_;
    std::atomic<bool> is_keyboard_reading_;

    // Value
    double joint_vel_cmd_;

    // Keyboard
    KeyboardReader input_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string node_name = "dbot_servo_keyboard_node";
    std::string joint_cmd_topic = "/servo_node/delta_joint_cmds";
    std::string cartesian_cmd_topic = "/servo_node/delta_twist_cmds";
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(false);
    auto node = std::make_shared<DbotServoKeyboardPublisher>(node_name, node_options, joint_cmd_topic, cartesian_cmd_topic);
    RCLCPP_INFO(node->get_logger(), "Dbot Keyboard Servo Start");
    rclcpp::spin(node);
    // auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    // executor->add_node(node);
    // executor->spin();
    RCLCPP_INFO(node->get_logger(), "Dbot Keyboard Servo Stop");
    
    rclcpp::shutdown();
    return 0;
}