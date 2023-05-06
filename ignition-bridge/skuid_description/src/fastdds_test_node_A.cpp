#include <functional>
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <iterator>

#include <custom_participant_listener.hpp>

#include <rclcpp/init_options.hpp>
#include <rclcpp/node_impl.hpp>
#include <rclcpp/context.hpp>

#include "rcl/visibility_control.h"

#include <rmw_fastrtps_shared_cpp/rmw_init.hpp>
#include "rmw/init_options.h"

#include <rcl/context.h>

#include <rmw/init.h>

#include <rmw_fastrtps_cpp/get_participant.hpp>
#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_context_impl.hpp"

#include <fastrtps/participant/Participant.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
// using fastrtps::rtps::RTPSParticipant

#include <rmw_fastrtps_cpp/get_publisher.hpp>
#include <rmw_fastrtps_cpp/init_rmw_context_impl.hpp>

#include <rmw_fastrtps_shared_cpp/participant.hpp>

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_context_impl.hpp"

using namespace std::chrono_literals;

struct fastdds_confs_publisher : public rclcpp::Node
{
    fastdds_confs_publisher(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions())
        : rclcpp::Node("log_publisher", Options)
    {

        this->declare_parameter<int>("new_parameter", 10);

        // ---- callback for parameter

        param_sub = std::make_shared<rclcpp::ParameterEventHandler>(this);

        param_cb = param_sub->add_parameter_callback("new_parameter", [this](const rclcpp::Parameter &p) -> void
                                                     { RCLCPP_INFO(rclcpp::get_logger("parameters"), "parameter_callback %s", p.get_name()); });

        msg_pub = this->create_publisher<std_msgs::msg::String>("log", rclcpp::SystemDefaultsQoS());
        auto handle = msg_pub->get_publisher_handle();

        msg_pub_second = this->create_publisher<std_msgs::msg::String>("log_second", rclcpp::SystemDefaultsQoS());

        timer_ = this->create_wall_timer(1s, [this]() -> void
                                         {
                                             msg.data = "Hello , count: " + std::to_string(this->count++);

                                             msg_second.data = "Hello Second, " + std::to_string(this->count);

                                             this->msg_pub->publish(msg);

                                             this->msg_pub_second->publish(msg_second); });

        rcl_publisher_t *rcl_pub = msg_pub->get_publisher_handle().get();

        rmw_publisher_t *rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);

        eprosima::fastdds::dds::DataWriter *dw = rmw_fastrtps_cpp::get_datawriter(rmw_pub);

        const eprosima::fastdds::dds::RTPSEndpointQos &ent = dw->get_qos().endpoint();

        RCLCPP_INFO(rclcpp::get_logger("[ endpoint ]"), "%ls", ent.history_memory_policy);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_second;
    rclcpp::TimerBase::SharedPtr timer_;
    int count;
    std_msgs::msg::String msg;
    std_msgs::msg::String msg_second;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_sub;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb;
};

int main(int argc, char *argv[])
{

    rclcpp::InitOptions init_op;
    init_op.set_domain_id(0);

    rclcpp::Context::SharedPtr context;


    // --------------------------------------------------------------
    // rmw_init_options_t* rmwInitOp; //= rcl_init_options_get_rmw_init_options(init_op.get_rcl_init_options());
    // rmwInitOp->enclave = "ParticipantName";
    // rmwInitOp->domain_id = 2;
    // --------------------------------------------------------------

    // rmw_context_t* rmwContext = new rmw_context_t();

    // rmw_init(rmwInitOp, rmwContext);

    // --------------------------------------------------------------
 
    context = std::make_shared<rclcpp::Context>();

    context->init(argc, argv, init_op);

    rmw_context_t* rmwContext = rcl_context_get_rmw_context(context->get_rcl_context().get());
    rmwContext->options.enclave = "Custom Participant";
    rmwContext->instance_id=2;
    rmwContext->options.instance_id = 5;

    

    context->add_on_shutdown_callback([]() -> void
                                      { RCLCPP_INFO(rclcpp::get_logger("main_publisher"), "SHUTING DOWN CONTEXT PUBLISHER, domain"); });

    rclcpp::NodeOptions Options;

    Options.context(context);

    rclcpp::init(argc, argv, init_op);

    std::shared_ptr<fastdds_confs_publisher> fastDssNode = std::make_shared<fastdds_confs_publisher>(Options);

    // -------------------------------------------------------------------------

    rcl_node_t *rcl_node = fastDssNode->get_node_base_interface()->get_rcl_node_handle();

    rmw_node_t *rmw_node = rcl_node_get_rmw_handle(rcl_node);

    auto impl = static_cast<CustomParticipantInfo *>(rmw_node->context->impl->participant_info);

    std::cout << impl->participant_->get_qos().wire_protocol().participant_id;

    eprosima::fastdds::dds::DomainParticipant *p =
        rmw_fastrtps_cpp::get_domain_participant(rmw_node);

    const eprosima::fastdds::dds::DomainParticipantQos& qos = (p->get_qos());

    std::cout << qos.wire_protocol().participant_id  << std::endl;

    auto list = std::make_unique<CustomDomainParticipantListener>();
    p->set_listener(list.get());

    rclcpp::spin(fastDssNode);

    // ----------------------------------------------------------------

    std::vector<std::string> names = p->get_participant_names();

    

    std::cout << "---NAMES---";
    std::copy(names.begin(), names.end(), std::ostream_iterator<std::string>(std::cout, " "));
    std::cout << "---NAMES---" << std::endl;

    std::cout << p->guid().guidPrefix.value << std::endl;

    // -------------------------------------------------------------------------

    rclcpp::shutdown();

    return 0;
}