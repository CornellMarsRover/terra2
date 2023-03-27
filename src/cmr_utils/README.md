# cmr_utils

@brief Package for utility functions and classes that are useful throughout the C++ codebase.

`cmr_debug.hpp` contains functions for debugging such as `CmrException` and
[assertions](@ref CMR_ASSERT).


`cmr::Clock` provides a clock interface which can be used in all code that requires
using some type of time keeper. The `cmr::Clock` interface can be mocked, for example
with the concrete `cmr::ProducerConsumerMockClock` but can also be given
the real system time, for example with the concrete `cmr::RealClock`.


The `ThreadWrapper` class provides an interface similar to `std::thread` but with 
RAII destruction policies similar to C++20's 
[`std::jthread`](https://en.cppreference.com/w/cpp/thread/jthread).

`cmr::string_format()` and `cmr::build_string()` provide ways to create 
formatted strings instead of repeated concatenation.

The @ref monad namespace contains helper function for monadic data types.
In particular, they are designed to transform the `std::optional` type
into something more akin to Rust's [`Option`](https://doc.rust-lang.org/std/option/).
They are all templated free functions that technically work with anything
satisfying the static interface detailed in @ref monad.