#include "cmr_utils/cmr_debug.hpp"

#include <atomic>

void default_assert_handler() { throw CmrAssertionException(); }

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

#ifndef NDEBUG
#include <fstream>

bool is_debugger_attached()
{
    std::ifstream sf("/proc/self/status");
    std::string s;
    while (sf >> s) {
        if (s == "TracerPid:") {
            int pid = 0;
            sf >> pid;
            return pid != 0;
        }
        std::getline(sf, s);
    }

    return false;
}

#else
bool is_debugger_attached() { return false; }
#endif