/** listener.cpp **/
 
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <std_msgs/msg/string.hpp>
 
using namespace std::chrono;
using std::placeholders::_1;
 
class Subscriber : public rclcpp::Node {
 
public:
    Subscriber() : Node("Subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&Subscriber::topic_callback, this, _1)
        );
    }
 
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard : '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
 
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}