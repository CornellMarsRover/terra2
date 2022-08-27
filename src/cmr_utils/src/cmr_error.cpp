#include "cmr_utils/cmr_error.hpp"

#include <atomic>
#include <csignal>

void default_assert_handler() { raise(SIGKILL); }

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::shared_ptr g_assert_handler =
    std::make_shared<assert_handler_t>(default_assert_handler);

assert_handler_t set_assert_handler(assert_handler_t handler) noexcept
{
    return *std::atomic_exchange(&g_assert_handler,
                                 std::make_shared<assert_handler_t>(handler));
    // safe to dereference because assert_handler pointer has static lifetime
}

assert_handler_t get_assert_handler() noexcept
{
    return *std::atomic_load(&g_assert_handler);
}