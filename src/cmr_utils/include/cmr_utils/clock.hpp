#pragma once
#include <type_traits>
/**
 * @file clock.hpp
 *
 * This provides a clock interface which can be mocked for testing
 *
 */

namespace cmr
{
/**
 * @brief A clock interface
 *
 * @tparam ClockType a clock type to use. Must have a static now() function that
 * returns a time of some kind. This time value must be equality comparable.
 */
template <typename ClockType>
class Clock
{
    static_assert(std::is_same_v<std::void_t<decltype(ClockType::now())>, void>,
                  "ClockType must have a static now() function");

  public:
    using time_point_t = std::remove_reference_t<decltype(ClockType::now())>;

    /**
     * @brief Get the current time
     *
     * @return time_point_t
     */
    virtual time_point_t now() noexcept = 0;

    /**
     * @brief Signals to the clock that  `time` has been received by the consumer.
     * This should be called after doing whatever action the consumer needed the time
     * for.
     *
     * @param time
     */
    virtual void register_time(time_point_t time) noexcept = 0;
};

/**
 * @brief A non-mocked clock
 *
 * @tparam ClockType
 */
template <typename ClockType>
class RealClock : public Clock<ClockType>
{
  public:
    typename Clock<ClockType>::time_point_t now() noexcept override
    {
        return ClockType::now();
    }

    void register_time(typename Clock<ClockType>::time_point_t) noexcept override {}
};
}  // namespace cmr