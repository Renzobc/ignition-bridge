#include <rclcpp/rclcpp.hpp>
// #include <include/rmw_fastrtps_cpp/get_participant.hpp>
#include <std_msgs/msg/string.hpp>
struct fastdds_confs_subscriber : public rclcpp::Node
{
    fastdds_confs_subscriber(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions()) : rclcpp::Node("log_subscriber", Options)
    {
        sub_msg = this->create_subscription<std_msgs::msg::String>("log", rclcpp::SystemDefaultsQoS(),
                                                                   [this](const std_msgs::msg::String &msg) -> void
                                                                   {
                                                                       RCLCPP_INFO(rclcpp::get_logger("msg subcriber"), "Gotten msg");
                                                                   });

        sub_msg_second = this->create_subscription<std_msgs::msg::String>("log_second", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::String &msg) -> void
                                                                          { RCLCPP_INFO(rclcpp::get_logger("msg second subcriber"), "Gotten second msg"); });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_second;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::InitOptions initOp;

    initOp.set_domain_id(2);

    rclcpp::Context::SharedPtr context;

    context = std::make_shared<rclcpp::Context>();

    context->init(argc, argv, initOp);

    context->add_on_shutdown_callback([]() -> void
                                      { RCLCPP_INFO(rclcpp::get_logger("main_subcriber"), "SHUTTING DOWN CONTEXT SUBSCRIBER"); });

    rclcpp::NodeOptions Options;

    Options.context(context);

    rclcpp::spin(std::make_shared<fastdds_confs_subscriber>(Options));
    rclcpp::shutdown();
    return 0;
}