#pragma once
#include "cmr_utils/cmr_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cmr::fabric
{
/**
 * @brief A type trait that checks if a type is a server provider which defines
 * a `server_t, `config_t`, and `make_return_t` type along with a `make_server()`
 * function.
 *
 * `server_t` is the type of the server returned from `make_server()`. I think of a
 * server as something that runs on a ROS executor, handling requests of some type by
 * invoking user defined callbacks. The name originates from service and action
 * servers, but this definition also incorporates topic subscribers and wall timers.
 *
 * `config_t` is the type of the configuration passed to `make_server()`.
 *
 * `make_return_t<T>` is a template class that provides a static `val` member which
 * is the default value of `T` that will be returned from the server callback if an
 * error occurs or the server is disabled. So if a callback returns `T`, and an error
 * occurs during that callback or a request is received when the server is inactive
 * but the callback is still active (the latter shouldn't really happen), then it
 * will return `make_return_t<T>::val`
 *
 * `make_server<T>()` will take a reference to the owning `LifecycleNode`, one or
 * more callback functions for the server to execute, and a `config_t` object to pass
 * more server specific config parameters. The function should also be a template,
 * templated on the type of the first callback function.
 *
 * For simplicity, this SFINAE concept does not enforce the `make_server` member
 *
 * @tparam T type to test if it statisfiest the requirements of a server policy
 */
template <typename T, typename = void>
struct IsServerType : std::false_type {
};

/**
 * @brief A type trait that checks if a type is a server provider.
 * This is the overload for when the type is a server provider
 * @see IsServerType
 */
template <typename T>
struct IsServerType<T, std::void_t<typename T::server_t, typename T::config_t,
                                   decltype(T::template make_return_t<int>::val)>>
    : std::true_type {
};

/** Helper for `IsServerType<T>` */
template <typename T>
constexpr bool is_server_type_v = IsServerType<T>::value;

/** Constructs the default return value for server callbacks */
template <typename T>
struct ServerDefaultArgumentConstructor {
    const inline static T val = T{};
};

/** Overload for `void` returning callbacks */
template <>
struct ServerDefaultArgumentConstructor<void> {
    const static char val = {};
};

/**
 * @brief A class that provides a subscription to LifecycleServer
 *
 * @tparam MessageT type of message to receive
 * @tparam AllocatorT allocator type
 * @tparam SubscribedT
 * @tparam ROSMessageT
 * @tparam MessageMemoryStrategyT
 */
template <typename MessageT, typename AllocatorT, typename SubscribedT,
          typename ROSMessageT, typename MessageMemoryStrategyT>
class SubscriptionServerPolicy
{
  public:
    using server_t = rclcpp::Subscription<MessageT, AllocatorT, SubscribedT,
                                          ROSMessageT, MessageMemoryStrategyT>;

    /** Configuration parameters for a topic subscription */
    struct Config {
        rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;

        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat;

        rclcpp::QoS qos;

        std::string topic_name;

        explicit Config(
            std::string topic, const rclcpp::QoS& c_qos,
            const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& c_options =
                (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()),
            typename MessageMemoryStrategyT::SharedPtr c_msg_mem_strat =
                (MessageMemoryStrategyT::create_default()))
            : options(c_options),
              msg_mem_strat(c_msg_mem_strat),
              qos(c_qos),
              topic_name(std::move(topic))
        {
        }

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
        explicit Config(std::string topic) : Config(std::move(topic), 10){};
    };

    using config_t = Config;

    template <typename T>
    using make_return_t = ServerDefaultArgumentConstructor<T>;

    template <typename CallbackT>
    static auto make_server(rclcpp_lifecycle::LifecycleNode& node_base,
                            CallbackT callback, Config config)
    {
        return node_base.create_subscription<MessageT>(
            config.topic_name, config.qos, std::forward<CallbackT>(callback),
            config.options, config.msg_mem_strat);
    }
};

/**
 * @brief Groups together configuration parameters for a service
 */
struct ServiceConfig {
    std::string service_name;
    rmw_qos_profile_t qos_profile;
    rclcpp::CallbackGroup::SharedPtr group;

    explicit ServiceConfig(std::string name,
                           rmw_qos_profile_t qos = rmw_qos_profile_services_default,
                           // NOLINTNEXTLINE(modernize-pass-by-value)
                           rclcpp::CallbackGroup::SharedPtr c_group = nullptr)
        : service_name(std::move(name)), qos_profile(qos), group(c_group)
    {
    }
};

/**
 * @brief A class that provides a service to LifecycleServer
 *
 * @tparam ServiceT
 */
template <typename ServiceT>
class ServiceServerPolicy
{
  public:
    using server_t = rclcpp::Service<ServiceT>;

    using config_t = ServiceConfig;

    template <typename T>
    using make_return_t = ServerDefaultArgumentConstructor<T>;

    /**
     * @brief Construct a new Service
     *
     * @tparam CallbackT service callback
     * @param node_base owning node which must outlive the service
     * @param topic_name service name
     * @param callback service callback called when a message is received
     * @param config configuration for the service
     * @return std::shared_ptr<rclcpp::Service<ServiceT>>
     */
    template <typename CallbackT>
    static auto make_server(rclcpp_lifecycle::LifecycleNode& node_base,
                            CallbackT callback, const config_t config)
    {
        return node_base.create_service<ServiceT>(config.service_name,
                                                  std::forward<CallbackT>(callback),
                                                  config.qos_profile, config.group);
    }
};

/**
 * @brief A class that provides a timer to LifecycleServer
 *
 * @tparam FuncT timer callback type
 * @tparam RepT timer period rep type
 * @tparam PeriodT timer period period type
 */
template <typename RepT, typename PeriodT>
struct WallTimerServerPolicy {
    using server_t = rclcpp::TimerBase;

    struct Config {
        std::chrono::duration<RepT, PeriodT> period;
    };

    using config_t = Config;

    template <typename T>
    using make_return_t = ServerDefaultArgumentConstructor<T>;

    template <typename CallbackT>
    static auto make_server(rclcpp_lifecycle::LifecycleNode& node_base,
                            CallbackT callback, const config_t config = config_t{})
    {
        return node_base.create_wall_timer(config.period,
                                           std::forward<CallbackT>(callback));
    }
};

/**
 * @brief Interface for any LifecycleServer.
 *
 * A lifecycle server is a server which registeres a callback with ROS
 * that is called when a certain event occurs during the spinning of the node.
 * It provides activation, deactivation, and polling of activation status.
 * When the lifecycle server is deactivated, it will not invoke any user callbacks.
 * A lifecycle server is initially deactivated until `activate()` is called.
 */
class GenericLifecycle
{
  public:
    virtual ~GenericLifecycle() = default;

    /** Activates the lifecycle server and registers the callback with ROS */
    virtual void activate() = 0;

    /** Deactivates the server and stops callbacks from being invoked */
    virtual void deactivate() = 0;

    virtual bool is_active() const = 0;
};

/**
 * @brief Creates a managed server that can be activated and
 * deactivated. The term server is used to mean something that runs on a ROS executor
 * and invokes user defined callbacks.
 *
 * Unlike the regular ROS counterparts, the constructor does not start the server.
 * Instead it must be explicitly activated by calling `activate()`. Furthermore, all
 * user-specified callbacks are wrapped in error handling logic so that any uncaught
 * exceptions will cause the error transition to trigger on the `FabricNode`. The
 * server can be manually deactivated with the `deactivate()` method, and you can
 * check the status of a server by querying the `is_active()` method.
 *
 * When the server is inactive, it will not invoke any user callbacks. It will also
 * not respond to any requests. If an error occurs, then the callback return will
 * be determined by `ServerT`'s `make_return_t` type alias.
 *
 * @tparam ServerT The server provider type. This type defines the types `server_t`,
 * `config_t, and `make_return_t` which are used to create the server and configure
 * it. It also must have a static method `make_server` that takes a
 * `rclcpp_lifecycle::LifecycleNode&`, one or more callbacks, and a
 * `config_t` and returns a `server_t`. It must satisfy the concept of a
 * `ServerPolicy`
 * @see IsServerPolicy
 */
template <class ServerT>
class LifecycleServer : public GenericLifecycle
{
    std::shared_ptr<typename ServerT::server_t> m_sub;

    bool m_active = false;

    unsigned m_error_count = 0;

    std::reference_wrapper<rclcpp_lifecycle::LifecycleNode> m_node;

    std::function<std::remove_reference_t<decltype(m_sub)>()> m_create_sub;

    static_assert(is_server_type_v<ServerT>, "ServerT must be a server policy type");

    /** How many repeated errors to tolerate before we yell at the user */
    static constexpr auto error_threshold = 3;

  public:
    using config_t = typename ServerT::config_t;
    using ptr_t = typename std::unique_ptr<LifecycleServer<ServerT>>;

    /**
     * @brief Construct a new Lifecycle Subscription object
     *
     * @param node_base parent node of the subscription. This node **MUST OUTLIVE**
     * the subscription
     * @param topic_name name of the topic to subscribe to
     * @param qos quality of service settings
     * @param callback callback when a message is received
     * @param error_callback callback when an error occurs in the subscription
     *  callback. For fabric nodes this should probably be
     * `FabricNode::error_transition()`
     * @param options optional subscription options
     * @param msg_mem_strat optional message memory strategy
     */
    template <typename ErrorCallbackT, typename... CallbackT>
    // NOLINTNEXTLINE(readability-function-size)
    LifecycleServer(rclcpp_lifecycle::LifecycleNode& node_base,
                    ErrorCallbackT&& error_callback,
                    const typename ServerT::config_t& config,
                    CallbackT&&... callbacks)
        : m_sub(nullptr),
          m_node(node_base),
          m_create_sub(
              [args = std::make_tuple(
                   std::ref(node_base),
                   create_function_wrapper(
                       std::forward<CallbackT>(callbacks),
                       std::forward<ErrorCallbackT>(error_callback),
                       std::unique_ptr<typename rclcpp::function_traits::
                                           function_traits<CallbackT>::arguments>{},
                       ServerT::template make_return_t<
                           typename rclcpp::function_traits::function_traits<
                               CallbackT>::return_type>::val)...,
                   config)]() {
                  // make_server is templated only on the first callback type
                  // this is all we need for now, but one could theoretically
                  // make changes to support all callback types as template
                  // parameters
                  return std::apply(ServerT::template make_server<
                                        std::tuple_element_t<1, decltype(args)>>,
                                    args);
              })
    {
    }

    LifecycleServer(const LifecycleServer&) = delete;
    LifecycleServer& operator=(const LifecycleServer&) = delete;

    // Cannot move because the subscription captures the `this` pointer
    // we could avoid this by making `m_active` a pointer, but generally it's
    // probably more effecient to just make a pointer to the server
    LifecycleServer(LifecycleServer&& other) noexcept = delete;
    LifecycleServer& operator=(LifecycleServer&& other) noexcept = delete;

    ~LifecycleServer() override = default;

    /**
     * @brief Activates the Lifecycle Subscription.
     * The on message received callback will not be invoked until this is called.
     */
    void activate() override
    {
        m_active = true;
        m_sub = m_create_sub();
    }

    /**
     * @brief Deactivates the Lifecycle Subscription.
     * The on message received callback will not be invoked after this is called.
     */
    void deactivate() override
    {
        m_active = false;
        m_sub.reset();
        m_error_count = 0;
    }

    bool is_active() const override { return m_active; }

  private:
    /**
     * @brief Create a callback wrapper that will call the user callback with
     * proper error handling and activation logic.
     *
     * This function takes a unique_ptr to the arguments of the callback for
     * type deduction only. It is expected that a `nullptr` of the correct type
     * is passed in.
     *
     * @tparam CallbackT type of the user callback
     * @tparam ErrorFuncT type of the error callback which is invoked if the user
     * callback throws
     * @tparam ArgsT type of the arguments to the user callback
     * @param callback user callback
     * @param error_cb error callback which is invoked if the user callback throws
     *   For fabric nodes this should probably be `FabricNode::error_transition()`
     * @param args_ptr unique_ptr to the arguments of the user callback. This is
     *  only used for type deduction
     * @param error_val value to return if the user callback throws or
     * the server is not active
     * @return callable object that wraps `callback`
     */
    template <typename CallbackT, typename ErrorFuncT, typename ReturnT,
              typename... ArgsT>
    auto create_function_wrapper(CallbackT&& callback, ErrorFuncT&& error_cb,
                                 std::unique_ptr<std::tuple<ArgsT...>>&&,
                                 ReturnT error_val)
    {
        return [this, callback = std::forward<CallbackT>(callback),
                error_cb = std::forward<ErrorFuncT>(error_cb),
                error_val](ArgsT... args) {
            if (m_active) {
                // m_active shouldn't be false, but check this as a sanity check
                // in case ROS copied the subscriber shared_ptr
                try {
                    return callback(std::forward<ArgsT>(args)...);
                } catch (const cmr_catch_t& e) {
                    RCLCPP_ERROR(m_node.get().get_logger(),
                                 "An error occurred in a server callback: '%s'",
                                 e.what());
                    if (++m_error_count > error_threshold) {
                        RCLCPP_ERROR(
                            m_node.get().get_logger(),
                            "More than %u uncaught exceptions occurred in "
                            "the same server callback. This happens when a "
                            "callback repeatedly errors, which should only "
                            "happen in extremely exceptional circumstances. "
                            "Exceptions should not be used for normal "
                            "control flow, and the restart behavior here "
                            "should not be relied upon for correct logic. Repeated "
                            "failures means there is a bug in your code.",
                            error_threshold);
                    }
                    error_cb();
                }
            }
            using return_type_t = std::remove_reference_t<decltype(callback(
                std::forward<ArgsT>(args)...))>;
            if constexpr (!std::is_same_v<return_type_t, void>) {
                return error_val;
            } else {
                // disable compiler warning of unused parameter for void functions
                (void)error_val;
            }
        };
    }
};

/**
 * @brief Creates a managed topic subscriber that can be activated and deactivated.
 * Other than `activate()`, `deactivate()`, and uncaught exceptions triggering a
 * fabric node transition instead of crashing, the this class provides a similar and
 * semantics as `rclcpp::Subscription`.
 *
 * @tparam MessageT The message type to subscribe to
 * @tparam AllocatorT
 * @tparam rclcpp::TypeAdapter<MessageT>::custom_type
 * @tparam rclcpp::TypeAdapter<MessageT>::ros_message_type
 * @tparam rclcpp::message_memory_strategy::
 * MessageMemoryStrategy<ROSMessageT, AllocatorT>
 */
template <typename MessageT, typename AllocatorT = std::allocator<void>,
          typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
          typename ROSMessageT =
              typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
          typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::
              MessageMemoryStrategy<ROSMessageT, AllocatorT>>
using LifecycleSubscription =
    LifecycleServer<SubscriptionServerPolicy<MessageT, AllocatorT, SubscribedT,
                                             ROSMessageT, MessageMemoryStrategyT>>;

/**
 * @brief Creates a managed service server that can be activated and deactivated.
 * Other than `activate()`, `deactivate()`, and uncaught exceptions triggering a
 * fabric node transition instead of crashing, this class provides a similar
 * and semantics as `rclcpp::Service`.
 *
 * @tparam ServiceT The service type to subscribe to
 */
template <typename ServiceT>
using LifecycleService = LifecycleServer<ServiceServerPolicy<ServiceT>>;

/**
 * @brief Creates a managed wall timer that can be activated and deactivated.
 * Other than `activate()`, `deactivate()`, and uncaught exceptions triggering a
 * fabric node transition instead of crashing, this class provides a similar
 * and semantics as `rclcpp::WallTimer`.
 *
 * @tparam TimerArgT timer callback type argument type. Either `void` or `TimerBase&`
 * @tparam RepT timer period rep type
 * @tparam PeriodT timer period period type
 */
template <typename RepT, typename PeriodT>
using LifecycleTimer = LifecycleServer<WallTimerServerPolicy<RepT, PeriodT>>;

/**
 * @brief Any lifecycle timer
 */
using GenericLifecycleTimer = GenericLifecycle;

}  // namespace cmr::fabric