#include <gtest/gtest.h>

#include <optional>

#include "cmr_utils/cmr_error.hpp"
#include "cmr_utils/cmr_monad.hpp"

TEST(Monad, optionalIsAMondad)
{
    std::optional<int> maybe_int = {5};
    static_assert(monad::IsMonadLikeV<std::optional<int>>);
    auto r = monad::map(maybe_int, [](int i) { return i + 10; });
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 15);

    ASSERT_EQ(monad::value_or(maybe_int, 20), 5);

    std::optional<void*> maybe_ptr = {};
    ASSERT_EQ(monad::value_or_else(maybe_ptr, []() { return nullptr; }), nullptr);
}

TEST(Monad, errorIsAMonad)
{
    static_assert(monad::IsMonadLikeV<Result<int>>);
    Result<int> res = {5};
    ASSERT_TRUE(res.has_value());
    constexpr auto num = 0xFFFFFFFFFF;
    ASSERT_EQ(monad::bind(res, [](auto) { return Result<long long>(num); }).value(),
              num);
    ASSERT_TRUE(res.maybe_get_val().has_value());
}

auto test_fun()
{
    return make_opaque_result<int>(BasicError{ErrorCode::OpaqueError});
}

TEST(Error, basicErrorToError) { ASSERT_TRUE(test_fun().has_error()); }