#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <unicorecomm_gps/UnicorecommGpsNode.hpp>

int main(int argc, char **argv) {

    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    const auto node = std::make_shared<unicorecomm_gps_node::UnicorecommGpsNode>(rclcpp::NodeOptions());

    RCLCPP_INFO(node->get_logger(), "启动 %s 节点...", node->get_name());

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "关闭 %s 节点...", node->get_name());

    rclcpp::shutdown();

    return 0;
}
