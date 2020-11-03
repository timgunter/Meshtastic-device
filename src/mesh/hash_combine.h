#pragma once

#include <functional>

template<typename T, typename... Tail>
struct hash_combine
{
    std::size_t operator()(T const &value, Tail... tail) const
    {
        // Based upon boost
        auto const hash = std::hash<T>()(value);
        return hash ^ (hash_combine<Tail...>()(tail...) + 0x9e3779b9 + (hash<<6) + (hash>>2));
    }
};

// Partial specialization to end variadic template recursion
template<typename T>
struct hash_combine<T>
{
    std::size_t operator()(T const &value) const
    {
        return std::hash<T>()(value);
    }
};

// Utility function to combine hashes using type deduction instead of specifying template parameters explicitly
template<typename... Args> inline
std::size_t make_hash(Args... args)
{
    return hash_combine<Args...>()(args...);
}
