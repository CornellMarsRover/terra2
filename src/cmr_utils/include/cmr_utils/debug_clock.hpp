#pragma once
#include <condition_variable>
#include <mutex>

#include "cmr_utils/clock.hpp"

namespace cmr
{
/**
 * @brief A clock that can be mocked for testing.
 *
 * This clock is designed to work with 1 producer and 1 or more consumers in
 * different threads. The producer is the thread that is setting the time, and the
 * consumers are the threads reading the time.
 *
 * @tparam ClockType
 */
template <typename ClockType>
class ProducerConsumerMockClock : public Clock<ClockType>
{
    using time_point_t = typename Clock<ClockType>::time_point_t;
    time_point_t m_time;
    bool m_mocked = false;
    time_point_t m_seen_check_time;
    std::mutex m_mock_mutex;
    std::mutex m_seen_check_mutex;
    std::condition_variable m_mock_cv;

  public:
    typename Clock<ClockType>::time_point_t now() noexcept override;

    /**
     * @brief Set the time to be returned by the clock
     * Thread safe
     *
     * @param time
     */
    void set_time(typename Clock<ClockType>::time_point_t time) noexcept;

    /**
     * @brief Set the time and waits until a consumer registers it.
     *
     * This function is not safe to be called when using multiple producers.
     * One producer could set the time and wait and another producer could overwrite
     * that time before any consumer sees the first time. This would leave the first
     * producer waiting forever.
     *
     * This function blocks the calling thread
     *
     * @param time
     */
    void set_time_and_wait(typename Clock<ClockType>::time_point_t time) noexcept;

    void register_time(
        typename Clock<ClockType>::time_point_t time) noexcept override;
};
}  // namespace cmr

#include "cmr_utils/debug_clock.inl"