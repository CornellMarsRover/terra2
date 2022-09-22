#pragma once
#include <functional>
#include <optional>
#include <thread>

#include "cmr_utils/cmr_debug.hpp"

/**
 * @brief Defers starting the thread until `start` is called
 *
 */
class DeferredCreationPolicy
{
  private:
    std::function<void(void)> m_func;
    std::optional<std::thread> m_thread;

  public:
    explicit DeferredCreationPolicy(std::function<void(void)> func)
        : m_func(std::move(func)), m_thread()
    {
    }

    void start()
    {
        if (!m_thread.has_value()) {
            m_thread.emplace(m_func);
        }
    }

    bool is_running() const { return m_thread.has_value(); }

  protected:
    auto& thread() { return m_thread; }
};

/**
 * @brief Starts the thread in the constructor
 */
class InstantCreationPolicy
{
  private:
    std::optional<std::thread> m_thread;

  public:
    explicit InstantCreationPolicy(std::function<void(void)> func)
        : m_thread(std::move(func))
    {
    }

  protected:
    auto& thread() { return m_thread; }
};

/**
 * @brief Joins the thread upon destruction
 *
 */
class JoinDestructionPolicy
{
  public:
    static void destroy(std::optional<std::thread>& thread)
    {
        if (thread.has_value() && thread->joinable()) {
            thread->join();
        }
    }
};

/**
 * @brief Detached the thread upon destruction
 *
 */
class DetatchDestructionPolicy
{
  public:
    static void destroy(std::optional<std::thread>& thread)
    {
        if (thread.has_value() && thread->joinable()) {
            thread->detach();
        }
    }
};

/**
 * @brief The ThreadWrapper provides RAII and other features for std::thread
 *
 * @tparam DestructionPolicy a class which provides a static `destroy` method that
 * acceptes an `std::optional<std::thread>&` that will be called upon destruction
 * @tparam CreationPolicy a class which provides a protected `thread()` method which
 * returns an `std::optional<std::thread>&` and a constructor that takes a callable
 * object that will be called when the ThreadWrapper is created
 */
template <typename DestructionPolicy, typename CreationPolicy>
class ThreadWrapper : public CreationPolicy
{
  public:
    explicit ThreadWrapper(std::function<void(void)> func)
        : CreationPolicy(std::move(func))
    {
    }

    ~ThreadWrapper() { DestructionPolicy::destroy(this->thread()); }
    ThreadWrapper(const ThreadWrapper&) = delete;
    ThreadWrapper& operator=(const ThreadWrapper&) = delete;

    ThreadWrapper(ThreadWrapper&&) noexcept = default;
    ThreadWrapper& operator=(ThreadWrapper&&) noexcept = default;

    inline void join()
    {
        CMR_ASSERT(this->thread().has_value());
        if (this->thread()->joinable()) {
            this->thread()->join();
        }
    }

    inline auto get_id() const
    {
        CMR_ASSERT(this->thread().has_value());
        return this->thread()->get_id();
    }

    inline void detach()
    {
        CMR_ASSERT(this->thread().has_value());
        this->thread()->detach();
    }

    inline bool joinable() const
    {
        CMR_ASSERT(this->thread().has_value());
        return this->thread()->joinable();
    }
};

/** A thread which starts immediately and joins on destruction */
using JThread = ThreadWrapper<JoinDestructionPolicy, InstantCreationPolicy>;