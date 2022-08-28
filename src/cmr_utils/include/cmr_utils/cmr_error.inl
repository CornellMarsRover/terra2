/// @file Inline definitions for cmr_error.hpp

[[noreturn]] inline void cmr_invalid(const char* file, unsigned line,
                                     const char* msg)
{
    CMR_LOG(FATAL, "CMR_INVALID triggered at %s:%u", file, line);
    CMR_LOG(FATAL, "%s", msg);
    get_assert_handler()();  // the assert handler could throw an exception
    std::terminate();
    // To use cmr_invalid to end control paths (ie. need not explicitly return from
    // a function after calling it), we must make sure we actually abort
    // Returning from a [[noreturn]] function is undefined behavior
}

namespace monad
{
template <typename T, typename Func>
constexpr auto bind(T&& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(f(monad.value()))>>
{
    if (monad.has_value()) {
        return f(std::forward<decltype(monad.value())>(monad.value()));
    } else {
        return std::remove_reference_t<decltype(f(monad.value()))>{};
    }
}

template <template <typename> class T, typename U, typename Func>
constexpr auto map(const T<U>& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T<U>>,
                        T<std::remove_reference_t<decltype(f(monad.value()))>>>
{
    if (monad.has_value()) {
        return T<std::remove_reference_t<decltype(f(monad.value()))>>{
            f(monad.value())};
    } else {
        return {};
    }
}

template <typename T, typename U>
constexpr auto value_or(T&& monad, U&& default_val) noexcept
    -> std::enable_if_t<is_monad_like_v<T>, U>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return default_val;
    }
}

template <typename T, typename Func>
constexpr auto value_or_else(T&& monad, Func f) noexcept(noexcept(f()))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(monad.value())>>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return f();
    }
}
}  // namespace monad