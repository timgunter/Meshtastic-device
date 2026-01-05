#pragma once

#include <memory>
#include <mutex>
#include <utility>

#ifndef ARCH_NRF52

namespace {
    using std::once_flag;
    using std::call_once;
}

// nrf52840 does not support std::call_once, so we use our own implementation
#else // ARCH_NRF52
#include <atomic>

namespace {

    struct once_flag {
        /// nrf52840 does not seem to have std::mutex either
        std::atomic_bool m_started{ false};
        std::atomic_bool m_finished{false};
        bool       m_flag{false};
    };

    template<typename Callable, typename... Args>
    void call_once(once_flag &flag, Callable &&func, Args &&... args) {
        bool expected = false;
        if(flag.m_started.compare_exchange_strong(expected, true, std::memory_order_release, std::memory_order_relaxed)) {
            func(std::forward<Args>(args)...);
            flag.m_finished = true;
            return;
        }

        while(!flag.m_finished.load())
            (void) 0;
    }
}
#endif

/// Specialize to define custom initialization behavior for the singleton instance.
template<typename T> struct SingletonInitor { void operator()(T &t) const {} };

/// Initor for use with classes that have static member function named "setDefault()"
template<typename T>
struct SingletonInitorStaticSetDefault{
    void operator()(T &t) const {
        // No RTTI
        LOG_DEBUG("%s singleton instance created and static initialized", __func__);
        T::setDefault();
    }
};

/// Initor for use with classes that have non-static member function named "setDefault()"
template<typename T>
struct SingletonInitorMemberSetDefault {
    void operator()(T &t) const {
        // No RTTI
        LOG_DEBUG("%s singleton instance created and member initialized", __func__);
        t.setDefault();
    }
};

template<typename T, typename Initor = SingletonInitor<T>>
struct Singleton {
    template<typename... Args>
    static T &getInstance(Initor const initor = Initor{}, Args &&... args) {
#       if __cplusplus >= 201402L // c++14 or later
            static auto singleton = std::make_unique<T>(std::forward<Args>(args)...);
#       else
            static auto singleton = std::make_shared<T>(std::forward<Args>(args)...);
#       endif
        static once_flag flag;
        call_once(flag, initor, *singleton);
        return *singleton;
    }
};

/// Singletons that assume T has static or member function named "setDefault()" for initialization.
template<typename T> using SingletonStaticSetDefault = Singleton<T, SingletonInitorStaticSetDefault<T>>;
template<typename T> using SingletonMemberSetDefault = Singleton<T, SingletonInitorMemberSetDefault<T>>;
