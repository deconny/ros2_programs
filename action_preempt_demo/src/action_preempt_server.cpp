#include <chrono>
#include <future>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/action/fibonacci.hpp"
#include "ThreadPool.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

class ActionPreemptServer : public rclcpp::Node
{
public:
    ActionPreemptServer()
        : rclcpp::Node("action_preempt_node")
        , thread_pool_(4)
    {}

    void init()
    {
        action_server_ = std::make_shared<nav2_util::SimpleActionServer<Fibonacci>>(
            shared_from_this(), "fibonacci", std::bind(&ActionPreemptServer::execute, this));
        action_server_->activate();
    }

private:
    void execute()
    {
        auto goal = action_server_->get_current_goal();
        RCLCPP_INFO(get_logger(), "Executing goal order %d", goal->order);

        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();
        sequence_.clear(); // Clear previous sequence
        sequence_.push_back(0);
        sequence_.push_back(1);

        // 将计算任务放入线程池
        auto future = thread_pool_.enqueue(
            [this, goal, feedback, result]() { return calculate_fibonacci_sequence(goal->order, feedback, result); });

        // 等待任务完成并处理结果
        bool success = future.get();
        if (success)
        {
            RCLCPP_INFO(get_logger(), "Goal order %d completed", (int)result->sequence.size());
            action_server_->succeeded_current(result);
        }
        else
        {
            action_server_->terminate_all(result);
        }
    }

    bool calculate_fibonacci_sequence(int order,
        std::shared_ptr<Fibonacci::Feedback> feedback,
        std::shared_ptr<Fibonacci::Result> result)
    {
        rclcpp::Rate loop_rate(1);
        int i = 1; // Initialize counter

        while (i < order && rclcpp::ok())
        {
            // Check for cancellation
            if (action_server_->is_cancel_requested() || !action_server_->is_server_active())
            {
                result->sequence = sequence_;
                action_server_->terminate_all();
                return false; // 返回失败
            }

            // Check for preemption
            if (action_server_->is_preempt_requested())
            {
                action_server_->accept_pending_goal();
                auto goal = action_server_->get_current_goal();
                RCLCPP_INFO(get_logger(), "Executing goal order %d", goal->order);
                feedback.reset(new Fibonacci::Feedback());
                result.reset(new Fibonacci::Result());
                sequence_.clear(); // Clear previous sequence
                sequence_.push_back(0);
                sequence_.push_back(1);
                i = 1; // Reset counter
                continue;
            }

            // Update the sequence
            sequence_.push_back(sequence_[i] + sequence_[i - 1]);

            // Update result sequence
            result->sequence = sequence_;

            // Publish feedback
            feedback->sequence = sequence_;
            action_server_->publish_feedback(feedback);
            loop_rate.sleep();

            ++i; // Increment counter
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            result->sequence = sequence_;
            return true; // 返回成功
        }

        return false; // 返回失败
    }

    std::shared_ptr<nav2_util::SimpleActionServer<Fibonacci>> action_server_;
    std::vector<int> sequence_; // Store Fibonacci sequence
    ThreadPool thread_pool_;    // Thread pool instance
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionPreemptServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}