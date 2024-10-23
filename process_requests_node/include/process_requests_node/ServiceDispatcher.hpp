#ifndef SERVICE_DISPATCHER_HPP
#define SERVICE_DISPATCHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "ThreadPool.hpp"

class ServiceDispatcher
{
public:
    static ServiceDispatcher &instance()
    {
        static ServiceDispatcher instance(4);
        return instance;
    }

    ServiceDispatcher(const ServiceDispatcher &) = delete;
    ServiceDispatcher &operator=(const ServiceDispatcher &) = delete;

    template<typename ServiceT>
    void registerService(const std::string &service_name)
    {
        std::lock_guard<std::mutex> lock(service_mutex_);
        if (service_clients_.find(service_name) != service_clients_.end())
        {
            RCLCPP_INFO(rclcpp::get_logger(service_name + "_node"), "Service '%s' is already registered.", service_name.c_str());
            return;
        }
        auto node = std::make_shared<rclcpp::Node>(service_name + "_node");
        auto client = node->create_client<ServiceT>(service_name);
        service_clients_[service_name] = {node, client};
        RCLCPP_INFO(node->get_logger(), "Service '%s' registered successfully.", service_name.c_str());
    }

    void unregisterService(const std::string &service_name)
    {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto it = service_clients_.find(service_name);
        if (it != service_clients_.end())
        {
            RCLCPP_INFO(it->second.node->get_logger(), "Service '%s' is being removed.", service_name.c_str());
            service_clients_.erase(it);
        }
    }

    template<typename ServiceT, typename RequestT, typename ResponseT>
    void sendAsyncRequest(const std::string &service_name,
        std::shared_ptr<RequestT> request,
        std::function<void(std::shared_ptr<ResponseT>)> callback,
        const std::chrono::seconds &timeout = std::chrono::seconds::max())
    {
        registerService<ServiceT>(service_name);
        thread_pool_.enqueue([this, service_name, request, callback, timeout]() {
            auto [node, client] = getServiceClient<ServiceT>(service_name);
            if (!client)
            {
                RCLCPP_ERROR(node->get_logger(), "Service '%s' is not registered.", service_name.c_str());
                return;
            }

            if (!waitForService(client, node, service_name))
            {
                return;
            }

            sendAsyncRequestImpl<ServiceT, RequestT, ResponseT>(client, request, callback, node, service_name, timeout);
        });
    }

private:
    ServiceDispatcher(size_t thread_pool_size)
        : thread_pool_(thread_pool_size)
    {}

    struct RequestInfo
    {
        rclcpp::Node::SharedPtr node;
        rclcpp::ClientBase::SharedPtr client;
    };

    template<typename ServiceT>
    std::pair<std::shared_ptr<rclcpp::Node>, std::shared_ptr<rclcpp::Client<ServiceT>>> getServiceClient(
        const std::string &service_name)
    {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto it = service_clients_.find(service_name);
        if (it == service_clients_.end())
        {
            return {nullptr, nullptr};
        }
        return {it->second.node, std::static_pointer_cast<rclcpp::Client<ServiceT>>(it->second.client)};
    }

    template<typename ServiceT>
    bool waitForService(const std::shared_ptr<rclcpp::Client<ServiceT>> &client,
        const std::shared_ptr<rclcpp::Node> &node,
        const std::string &service_name)
    {
        while (!client->wait_for_service(std::chrono::seconds(3)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(
                    node->get_logger(), "Client interrupted while waiting for service '%s'.", service_name.c_str());
                return false;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service '%s' to appear...", service_name.c_str());
        }
        return true;
    }

    template<typename ServiceT, typename RequestT, typename ResponseT>
    void sendAsyncRequestImpl(const std::shared_ptr<rclcpp::Client<ServiceT>> &client,
        const std::shared_ptr<RequestT> &request,
        const std::function<void(std::shared_ptr<ResponseT>)> &callback,
        const std::shared_ptr<rclcpp::Node> &node,
        const std::string &service_name,
        const std::chrono::seconds &timeout)
    {
        auto future_result = client->async_send_request(request);

        auto result = rclcpp::spin_until_future_complete(node, future_result, timeout);
        if (result == rclcpp::FutureReturnCode::SUCCESS)
        {
            try
            {
                callback(future_result.get());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "Exception in callback: %s", e.what());
            }
        }
        else if (result == rclcpp::FutureReturnCode::TIMEOUT)
        {
            RCLCPP_ERROR(node->get_logger(), "Service call to '%s' timed out.", service_name.c_str());
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service call to '%s' failed.", service_name.c_str());
        }
    }

    ThreadPool thread_pool_;
    std::unordered_map<std::string, RequestInfo> service_clients_;
    std::mutex service_mutex_;
};

#endif // SERVICE_DISPATCHER_HPP