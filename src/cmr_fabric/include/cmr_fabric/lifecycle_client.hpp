#pragma once
#include <rclcpp/rclcpp.hpp>

namespace cmr::fabric
{
/**
 * @brief Lifecycle Client is just like a `rclcpp::Client` but it can be activated
 * and deactivated.
 *
 * When inactive, the client will not be able to send requests.
 *
 * @tparam ServiceT the type of the service
 */
template <typename ServiceT>
class LifecycleClient : public rclcpp::ClientBase
{
    using client_t = rclcpp::Client<ServiceT>;
    std::unique_ptr<rclcpp::Client<ServiceT>> m_client;
    bool m_active = false;

  public:
    using SharedRequest = typename client_t::SharedRequest;
    using SharedResponse = typename client_t::SharedResponse;

    using Promise = std::promise<SharedResponse>;
    using PromiseWithRequest =
        std::promise<std::pair<SharedRequest, SharedResponse>>;

    using SharedPromise = std::shared_ptr<Promise>;
    using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

    using SharedFuture = std::shared_future<SharedResponse>;
    using SharedFutureWithRequest =
        std::shared_future<std::pair<SharedRequest, SharedResponse>>;

    using CallbackType = std::function<void(SharedFuture)>;
    using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;

    RCLCPP_SMART_PTR_DEFINITIONS(LifecycleClient)

    /**
     * @brief Construct a new Lifecycle Client object
     *
     * @param node_base the owning node which **MUST OUTLIVE** this client
     * @param node_graph
     * @param service_name the name of the service to connect to
     * @param client_options middleware client options
     */
    LifecycleClient(
        rclcpp::node_interfaces::NodeBaseInterface* node_base,
        rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
        const std::string& service_name, rcl_client_options_t& client_options)
        : ClientBase(node_base, node_graph),
          m_client(std::make_unique<client_t>(node_base, node_graph, service_name,
                                              client_options))
    {
    }

    ~LifecycleClient() override = default;

    bool take_response(typename ServiceT::Response& response_out,
                       rmw_request_id_t& request_header_out)
    {
        return this->take_type_erased_response(&response_out, request_header_out);
    }

    std::shared_ptr<void> create_response() override
    {
        return std::shared_ptr<void>(new typename ServiceT::Response());
    }

    std::shared_ptr<rmw_request_id_t> create_request_header() override
    {
        return m_client->create_request_header();
    }

    void handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                         std::shared_ptr<void> response) override
    {
        m_client->handle_response(request_header, response);
    }

    /**
     * @brief Sends a request to the service
     *
     * @param request the request to send
     * @return a future that will be fulfilled when the response is received
     *  or an empty optional if the client is inactive
     */
    auto async_send_request(SharedRequest request) -> std::optional<
        std::remove_reference_t<decltype(m_client->async_send_request(request))>>
    {
        if (m_active) {
            return m_client->async_send_request(request);
        } else {
            return {};
        }
    }

    /**
     * @brief Activate the client.
     * The client will not sent requests until it is activated.
     */
    inline void activate() { m_active = true; }

    /**
     * @brief Deactivate the client.
     * The client will not sent requests after it is deactivated.
     */
    inline void deactivate() { m_active = false; }

    inline bool is_active() { return m_active; }
};
}  // namespace cmr::fabric