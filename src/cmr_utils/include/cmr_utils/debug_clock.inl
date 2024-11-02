#pragma once
#include "cmr_utils/debug_clock.hpp"

namespace cmr
{

template <typename ClockType>
typename Clock<ClockType>::time_point_t
ProducerConsumerMockClock<ClockType>::now() noexcept
{
    std::unique_lock lock(m_mock_mutex);
    if (m_mocked) {
        return m_time;
    } else {
        lock.unlock();
        return ClockType::now();
    }
}

template <typename ClockType>
void ProducerConsumerMockClock<ClockType>::set_time(
    typename Clock<ClockType>::time_point_t time) noexcept
{
    std::unique_lock lock(m_mock_mutex);
    m_time = time;
    m_mocked = true;
}

template <typename ClockType>
void ProducerConsumerMockClock<ClockType>::set_time_and_wait(
    typename Clock<ClockType>::time_point_t time) noexcept
{
    std::unique_lock lock(m_mock_mutex);
    m_time = time;
    m_mocked = true;
    m_mock_mutex.unlock();
    std::unique_lock lock2(m_seen_check_mutex);
    m_mock_cv.wait(lock2, [this, time] { return m_seen_check_time == time; });
}

template <typename ClockType>
void ProducerConsumerMockClock<ClockType>::register_time(
    typename Clock<ClockType>::time_point_t time) noexcept
{
    std::unique_lock lock2(m_seen_check_mutex);
    m_seen_check_time = time;
    lock2.unlock();
    m_mock_cv.notify_all();
}

}  // namespace cmr