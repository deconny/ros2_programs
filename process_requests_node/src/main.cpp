#include "example_interfaces/srv/add_two_ints.hpp"
#include "process_requests_node/ServiceDispatcher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("process_requests_node");

    using AddTwoIntsService = example_interfaces::srv::AddTwoInts;

    std::string add_service_name_one = "add_two_ints_one";
    // 创建请求
    auto request_add_one = std::make_shared<AddTwoIntsService::Request>();
    request_add_one->a = 7;
    request_add_one->b = 5;
    ServiceDispatcher::instance().sendAsyncRequest<AddTwoIntsService>(
        add_service_name_one,
        request_add_one,
        [node, add_service_name_one](std::shared_ptr<AddTwoIntsService::Response> response) {
            if (response->sum)
            {
                RCLCPP_INFO(node->get_logger(),
                    "Service %s call succeeded, result: %ld",
                    add_service_name_one.c_str(),
                    response->sum);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Service call failed.");
            }
        });
    
    std::string add_service_name_two = "add_two_ints_two";
    // 处理请求
    auto request_add_two = std::make_shared<AddTwoIntsService::Request>();
    request_add_two->a = 11;
    request_add_two->b = 5;
    ServiceDispatcher::instance().sendAsyncRequest<AddTwoIntsService>(
        add_service_name_two,
        request_add_two,
        [node, add_service_name_two](std::shared_ptr<AddTwoIntsService::Response> response) {
            if (response->sum)
            {
                RCLCPP_INFO(node->get_logger(),
                    "Service %s call succeeded, result: %ld",
                    add_service_name_two.c_str(),
                    response->sum);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Service call failed.");
            }
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
