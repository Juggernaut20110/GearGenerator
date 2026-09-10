#pragma once

#include <stdexcept>

// This file intentionally uses only the Windows COM ABI.  No SOLIDWORKS
// type-library headers are required: the application is late-bound through
// IDispatch just like the Python reference implementation.

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <oaidl.h>
#include <oleauto.h>
#include <objbase.h>
#include <windows.h>

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace geargen::solidworks::com {

[[nodiscard]] std::string hresult_text(HRESULT value);
void check_hresult(HRESULT value, std::string_view operation);

class Error : public std::runtime_error {
public:
    Error(HRESULT value, std::string operation);

    [[nodiscard]] HRESULT hresult() const noexcept { return value_; }
    [[nodiscard]] const std::string& operation() const noexcept { return operation_; }

private:
    HRESULT value_{};
    std::string operation_;
};

class Bstr {
public:
    Bstr() noexcept = default;
    explicit Bstr(const wchar_t* value);
    explicit Bstr(const std::wstring& value);
    Bstr(const Bstr& other);
    Bstr& operator=(const Bstr& other);
    Bstr(Bstr&& other) noexcept;
    Bstr& operator=(Bstr&& other) noexcept;
    ~Bstr();

    [[nodiscard]] BSTR get() const noexcept { return value_; }
    [[nodiscard]] bool empty() const noexcept { return value_ == nullptr || *value_ == L'\0'; }

private:
    BSTR value_{};
};

[[nodiscard]] std::wstring utf8_to_wide(std::string_view value);
[[nodiscard]] std::string wide_to_utf8(std::wstring_view value);

template <typename T>
class Ptr {
public:
    Ptr() noexcept = default;
    explicit Ptr(T* value) noexcept : value_(value) {}
    Ptr(const Ptr& other) noexcept : value_(other.value_)
    {
        if (value_) value_->AddRef();
    }
    Ptr& operator=(const Ptr& other) noexcept
    {
        if (this == &other) return *this;
        reset(other.value_);
        return *this;
    }
    Ptr(Ptr&& other) noexcept : value_(other.value_)
    {
        other.value_ = nullptr;
    }
    Ptr& operator=(Ptr&& other) noexcept
    {
        if (this == &other) return *this;
        release();
        value_ = other.value_;
        other.value_ = nullptr;
        return *this;
    }
    ~Ptr() { release(); }

    void reset(T* value = nullptr) noexcept
    {
        if (value) value->AddRef();
        release();
        value_ = value;
    }
    [[nodiscard]] T* get() const noexcept { return value_; }
    [[nodiscard]] T* operator->() const noexcept { return value_; }
    [[nodiscard]] explicit operator bool() const noexcept { return value_ != nullptr; }

    template <typename U>
    [[nodiscard]] Ptr<U> query() const
    {
        Ptr<U> result;
        if (!value_) return result;
        U* queried = nullptr;
        check_hresult(value_->QueryInterface(__uuidof(U), reinterpret_cast<void**>(&queried)),
                      "QueryInterface");
        result = Ptr<U>(queried);
        return result;
    }

    [[nodiscard]] T* detach() noexcept
    {
        T* result = value_;
        value_ = nullptr;
        return result;
    }

private:
    void release() noexcept
    {
        if (value_) value_->Release();
        value_ = nullptr;
    }

    T* value_{};
};

class Apartment {
public:
    explicit Apartment(DWORD model = COINIT_APARTMENTTHREADED);
    Apartment(const Apartment&) = delete;
    Apartment& operator=(const Apartment&) = delete;
    ~Apartment();

private:
    bool initialized_{false};
};

class Variant {
public:
    Variant();
    explicit Variant(double value);
    explicit Variant(long value);
    explicit Variant(int value) : Variant(static_cast<long>(value)) {}
    explicit Variant(bool value);
    explicit Variant(const char* value);
    explicit Variant(std::string_view value);
    explicit Variant(const std::wstring& value);
    explicit Variant(IDispatch* value);

    Variant(const Variant& other);
    Variant& operator=(const Variant& other);
    Variant(Variant&& other) noexcept;
    Variant& operator=(Variant&& other) noexcept;
    ~Variant();

    [[nodiscard]] static Variant null_dispatch();
    [[nodiscard]] static Variant doubles(const std::vector<double>& values);
    [[nodiscard]] static Variant byref_i4_variant(long initial = 0);

    [[nodiscard]] const VARIANT& native() const noexcept { return value_; }
    [[nodiscard]] VARIANT& native() noexcept { return value_; }
    [[nodiscard]] VARTYPE type() const noexcept { return value_.vt; }
    [[nodiscard]] bool is_nullish() const noexcept;
    [[nodiscard]] double as_double() const;
    [[nodiscard]] long as_long() const;
    [[nodiscard]] bool as_bool() const;
    [[nodiscard]] std::string as_string() const;
    [[nodiscard]] std::vector<double> as_doubles() const;
    [[nodiscard]] std::vector<Ptr<IDispatch>> as_dispatches() const;
    [[nodiscard]] IDispatch* as_dispatch() const;
    [[nodiscard]] long& byref_i4_value() const;

private:
    void clear() noexcept;
    void copy_from(const Variant& other);
    void move_from(Variant&& other) noexcept;

    VARIANT value_{};
    std::shared_ptr<long> byref_i4_;
};

class Dispatch {
public:
    Dispatch() noexcept = default;
    explicit Dispatch(IDispatch* value) noexcept : value_(value) {}
    explicit Dispatch(Ptr<IDispatch> value) noexcept : value_(std::move(value)) {}

    [[nodiscard]] bool valid() const noexcept { return static_cast<bool>(value_); }
    [[nodiscard]] IDispatch* get() const noexcept { return value_.get(); }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }

    [[nodiscard]] DISPID id(std::string_view name) const;
    [[nodiscard]] Variant call(std::string_view name,
                               const std::vector<Variant>& arguments = {},
                               std::string_view operation = {}) const;
    [[nodiscard]] Variant get(std::string_view name,
                              std::string_view operation = {}) const;
    void put(std::string_view name, const Variant& value,
             std::string_view operation = {}) const;
    [[nodiscard]] Dispatch call_dispatch(std::string_view name,
                                         const std::vector<Variant>& arguments = {},
                                         std::string_view operation = {}) const;
    [[nodiscard]] Dispatch get_dispatch(std::string_view name,
                                        std::string_view operation = {}) const;

private:
    [[nodiscard]] Variant invoke(std::string_view name, WORD flags,
                                 const std::vector<Variant>& arguments,
                                 std::string_view operation) const;
    [[nodiscard]] Variant invoke_once(DISPID member, WORD flags,
                                      const std::vector<Variant>& arguments,
                                      std::string_view operation) const;

    Ptr<IDispatch> value_;
};

[[nodiscard]] Dispatch create_dispatch(std::string_view progid);

} // namespace geargen::solidworks::com

#else

// The mathematical/core build remains portable.  The real COM implementation
// is compiled only on Windows; these declarations keep the public header
// includable by a non-Windows documentation or analysis build.
namespace geargen::solidworks::com {
class Dispatch {};
class Variant {};
class Error : public std::runtime_error {
public:
    explicit Error(const std::string& message) : std::runtime_error(message) {}
};
class Apartment {};
} // namespace geargen::solidworks::com

#endif
