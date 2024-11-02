#include <gtest/gtest.h>

#include <optional>

#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/monad.hpp"

TEST(Monad, optionalIsAMonad)
{
    std::optional<int> maybe_int = {5};
    static_assert(monad::is_monad_like_v<std::optional<int>>);
    static_assert(!monad::is_monad_like_v<int>);
    auto r = monad::map(maybe_int, [](int i) { return i + 10; });
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 15);

    ASSERT_EQ(monad::value_or(maybe_int, 20), 5);

    std::optional<void*> maybe_ptr = {};
    ASSERT_EQ(monad::value_or_else(maybe_ptr, []() { return nullptr; }), nullptr);
}
void test_assert_handler() { throw std::invalid_argument(""); }

void invoke_assert() { CMR_ASSERT(false); }

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

TEST(Monad, mapChangeType)
{
    std::optional<std::string> opt = {"Hello World"};
    const auto str_len = monad::map(opt, [](const auto& str) { return str.size(); });
    ASSERT_EQ(str_len, opt.value().size());

    std::optional<int> opt2 = {};
    ASSERT_EQ(monad::value_or(opt2, 10), 10);
    ASSERT_EQ(monad::value_or_else(opt2, []() { return 100 * 100; }), 100 * 100);
}