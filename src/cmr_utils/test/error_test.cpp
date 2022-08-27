#include <gtest/gtest.h>

#include <optional>

#include "cmr_utils/cmr_error.hpp"

TEST(Monad, optionalIsAMondad)
{
    std::optional<int> maybe_int = {5};
    static_assert(monad::is_monad_like_v<std::optional<int>>);
    auto r = monad::map(maybe_int, [](int i) { return i + 10; });
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 15);

    ASSERT_EQ(monad::value_or(maybe_int, 20), 5);

    std::optional<void*> maybe_ptr = {};
    ASSERT_EQ(monad::value_or_else(maybe_ptr, []() { return nullptr; }), nullptr);
}
void test_assert_handler() { throw std::invalid_argument(""); }

void invoke_assert() { CMR_ASSERT(false, "TEST"); }

TEST(Error, changeAssertHandler)
{
    const auto old = set_assert_handler(test_assert_handler);
    ASSERT_THROW(invoke_assert(), std::invalid_argument);
    set_assert_handler(old);
}

TEST(Monad, bindTest)
{
    std::optional<int> res = {5};
    ASSERT_TRUE(res.has_value());
    constexpr auto num = 0xFFFFFFFFFF;
    ASSERT_EQ(monad::bind(res, [num](auto) { return std::optional<long long>(num); })
                  .value(),
              num);
    ASSERT_TRUE(res.has_value());
}