#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class ActionPreemptClient : public rclcpp::Node
{
public:
    ActionPreemptClient()
        : Node("fibonacci_client"), current_goal_index_(0)
    {
        // 配置目标的 order 值和对应的时间间隔
        goals_ = {
            {10, 6},
            {20, 6},
            {10, 6}
        };

        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ActionPreemptClient::send_next_goal, this));
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<int, int>> goals_;
    size_t current_goal_index_;

    void send_next_goal()
    {
        if (current_goal_index_ >= goals_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All goals sent.");
            this->timer_->cancel();
            return;
        }

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = goals_[current_goal_index_].first;

        RCLCPP_INFO(this->get_logger(), "Sending goal with order %d", goal_msg.order);

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ActionPreemptClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ActionPreemptClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ActionPreemptClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // 设置一个定时器，在指定的时间间隔后发送下一个目标
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(goals_[current_goal_index_].second),
            std::bind(&ActionPreemptClient::send_next_goal, this));

        current_goal_index_++;
    }

    void goal_response_callback(std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %d", feedback->sequence.back());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            RCLCPP_INFO(this->get_logger(), "Result size: %d", (int)result.result->sequence.size());
            for (auto number : result.result->sequence)
            {
                RCLCPP_INFO(this->get_logger(), "%d", number);
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<ActionPreemptClient>();
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}