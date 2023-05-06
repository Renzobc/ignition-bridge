#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace skuid
{
    using namespace std::placeholders;
    struct ign_localization_node : public rclcpp::Node
    {
        ign_localization_node(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions())
            : rclcpp::Node("ign_localization", Options)
        {
            
            // Initialize the transform broadcaster
            // tf_broadcast_static = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            tf_broadcast_dynamic = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            sub_dynamic = this->create_subscription<geometry_msgs::msg::TransformStamped>("skuid/pose", 10, std::bind(&ign_localization_node::broadcast_tf_dynamic, this, _1));
            // sub_static = this->create_subscription<geometry_msgs::msg::TransformStamped>("skuid/pose_static", 10, std::bind(&ign_localization_node::broadcast_tf_static, this, _1));
            msg_pub = this->create_publisher<std_msgs::msg::String>("log", rclcpp::SystemDefaultsQoS());
        }



        void broadcast_tf_dynamic(const geometry_msgs::msg::TransformStamped &msg)
        {
            bool flag = false;
            geometry_msgs::msg::TransformStamped transform{};
            //RCLCPP_INFO(this->get_logger(), "RECEIVED TRANSFORM %s TO %s", msg.header.frame_id.c_str(), msg.child_frame_id.c_str());

            if (msg.header.frame_id == "skuid" && msg.child_frame_id.find("wheel") != std::string::npos)
            {
                flag = true;
                transform.header.set__frame_id("base_link");

                std::string frame_name = msg.child_frame_id;
                frame_name = frame_name.substr(frame_name.find("/") + 1, frame_name.length());

                transform.set__child_frame_id(frame_name);

                transform.header.stamp = this->get_clock()->now();

                // if (frame_name.find("right_front")==std::string::npos)
                //     RCLCPP_INFO(this->get_logger(), "FRAME ID RECEIVED %s", frame_name.c_str());
            }

            if (msg.header.frame_id == "levels" && msg.child_frame_id == "skuid")
            {
                flag = true;
                transform.set__child_frame_id("base_link");
                transform.header.set__frame_id("odom");
                transform.header.stamp = msg.header.stamp;
            }

            if (flag)
            {
                try
                {
                    
                    transform.transform.translation.x = msg.transform.translation.x;
                    transform.transform.translation.y = msg.transform.translation.y;
                    transform.transform.translation.z = msg.transform.translation.z;

                    transform.transform.rotation.x = msg.transform.rotation.x;
                    transform.transform.rotation.y = msg.transform.rotation.y;
                    transform.transform.rotation.z = msg.transform.rotation.z;
                    transform.transform.rotation.w = msg.transform.rotation.w;
                    tf_broadcast_dynamic->sendTransform(transform);
                    std_msgs::msg::String msg;
                    msg.data = "sent robot pose transform";
                    msg_pub->publish(msg);
                }
                catch (tf2::TransformException &e)
                {
                    RCLCPP_INFO(this->get_logger(), "%s", e.what());
                }
            }
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_dynamic;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_static;
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_static;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_dynamic;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub;
        
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(skuid::ign_localization_node)
