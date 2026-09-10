#include "solidworks/com.hpp"

#ifdef _WIN32

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace geargen::solidworks::com {

namespace {

std::string operation_name(std::string_view operation, std::string_view fallback)
{
    return operation.empty() ? std::string(fallback) : std::string(operation);
}

bool can_retry_as_property(HRESULT value) noexcept
{
    return value == DISP_E_MEMBERNOTFOUND || value == DISP_E_TYPEMISMATCH ||
           value == DISP_E_BADPARAMCOUNT;
}

} // namespace

std::string hresult_text(HRESULT value)
{
    std::ostringstream output;
    output << "HRESULT 0x" << std::uppercase << std::hex
           << static_cast<unsigned long>(value);
    wchar_t* buffer = nullptr;
    const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER |
                        FORMAT_MESSAGE_FROM_SYSTEM |
                        FORMAT_MESSAGE_IGNORE_INSERTS;
    const DWORD length = FormatMessageW(flags, nullptr, static_cast<DWORD>(value), 0,
                                        reinterpret_cast<LPWSTR>(&buffer), 0, nullptr);
    if (length != 0 && buffer != nullptr) {
        std::wstring message(buffer, length);
        while (!message.empty() && (message.back() == L'\r' || message.back() == L'\n' ||
                                    message.back() == L' ')) {
            message.pop_back();
        }
        output << " (" << wide_to_utf8(message) << ")";
        LocalFree(buffer);
    }
    return output.str();
}

void check_hresult(HRESULT value, std::string_view operation)
{
    if (FAILED(value)) throw Error(value, std::string(operation));
}

Error::Error(HRESULT value, std::string operation)
    : std::runtime_error(operation + ": " + hresult_text(value)),
      value_(value), operation_(std::move(operation))
{
}

Bstr::Bstr(const wchar_t* value)
    : value_(SysAllocString(value == nullptr ? L"" : value))
{
    if (value_ == nullptr) throw std::bad_alloc();
}

Bstr::Bstr(const std::wstring& value)
    : Bstr(value.c_str())
{
}

Bstr::Bstr(const Bstr& other)
    : Bstr(other.value_ == nullptr ? L"" : other.value_)
{
}

Bstr& Bstr::operator=(const Bstr& other)
{
    if (this == &other) return *this;
    Bstr copy(other);
    std::swap(value_, copy.value_);
    return *this;
}

Bstr::Bstr(Bstr&& other) noexcept
    : value_(other.value_)
{
    other.value_ = nullptr;
}

Bstr& Bstr::operator=(Bstr&& other) noexcept
{
    if (this == &other) return *this;
    SysFreeString(value_);
    value_ = other.value_;
    other.value_ = nullptr;
    return *this;
}

Bstr::~Bstr()
{
    SysFreeString(value_);
}

std::wstring utf8_to_wide(std::string_view value)
{
    if (value.empty()) return {};
    const int length = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                           value.data(), static_cast<int>(value.size()),
                                           nullptr, 0);
    if (length <= 0) throw Error(HRESULT_FROM_WIN32(GetLastError()), "UTF-8 to UTF-16 conversion");
    std::wstring result(static_cast<std::size_t>(length), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                        static_cast<int>(value.size()), result.data(), length);
    return result;
}

std::string wide_to_utf8(std::wstring_view value)
{
    if (value.empty()) return {};
    const int length = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
                                           value.data(), static_cast<int>(value.size()),
                                           nullptr, 0, nullptr, nullptr);
    if (length <= 0) throw Error(HRESULT_FROM_WIN32(GetLastError()), "UTF-16 to UTF-8 conversion");
    std::string result(static_cast<std::size_t>(length), '\0');
    WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, value.data(),
                        static_cast<int>(value.size()), result.data(), length,
                        nullptr, nullptr);
    return result;
}

Apartment::Apartment(DWORD model)
{
    const HRESULT result = CoInitializeEx(nullptr, model);
    if (FAILED(result)) throw Error(result, "CoInitializeEx");
    initialized_ = true;
}

Apartment::~Apartment()
{
    if (initialized_) CoUninitialize();
}

Variant::Variant()
{
    VariantInit(&value_);
}

Variant::Variant(double value)
    : Variant()
{
    value_.vt = VT_R8;
    value_.dblVal = value;
}

Variant::Variant(long value)
    : Variant()
{
    value_.vt = VT_I4;
    value_.lVal = value;
}

Variant::Variant(bool value)
    : Variant()
{
    value_.vt = VT_BOOL;
    value_.boolVal = value ? VARIANT_TRUE : VARIANT_FALSE;
}

Variant::Variant(const char* value)
    : Variant(std::string_view(value == nullptr ? "" : value))
{
}

Variant::Variant(std::string_view value)
    : Variant(utf8_to_wide(value))
{
}

Variant::Variant(const std::wstring& value)
    : Variant()
{
    Bstr string(value);
    value_.vt = VT_BSTR;
    value_.bstrVal = SysAllocString(string.get());
    if (value_.bstrVal == nullptr) throw std::bad_alloc();
}

Variant::Variant(IDispatch* value)
    : Variant()
{
    value_.vt = VT_DISPATCH;
    value_.pdispVal = value;
    if (value_.pdispVal != nullptr) value_.pdispVal->AddRef();
}

Variant::Variant(const Variant& other)
    : Variant()
{
    copy_from(other);
}

Variant& Variant::operator=(const Variant& other)
{
    if (this == &other) return *this;
    clear();
    copy_from(other);
    return *this;
}

Variant::Variant(Variant&& other) noexcept
    : Variant()
{
    move_from(std::move(other));
}

Variant& Variant::operator=(Variant&& other) noexcept
{
    if (this == &other) return *this;
    clear();
    move_from(std::move(other));
    return *this;
}

Variant::~Variant()
{
    clear();
}

void Variant::clear() noexcept
{
    VariantClear(&value_);
    byref_i4_.reset();
    VariantInit(&value_);
}

void Variant::copy_from(const Variant& other)
{
    if (other.byref_i4_) {
        // A BYREF argument must point at the caller's storage so the out value
        // written by IDispatch is visible after the invocation.  The shared
        // owner keeps that storage alive across the temporary VARIANT copies
        // used to reverse COM argument order.
        byref_i4_ = other.byref_i4_;
        value_.vt = VT_BYREF | VT_I4;
        value_.plVal = byref_i4_.get();
        return;
    }
    check_hresult(VariantCopy(&value_, const_cast<VARIANT*>(&other.value_)), "VariantCopy");
}

void Variant::move_from(Variant&& other) noexcept
{
    byref_i4_ = std::move(other.byref_i4_);
    value_ = other.value_;
    if (byref_i4_) value_.plVal = byref_i4_.get();
    VariantInit(&other.value_);
}

Variant Variant::null_dispatch()
{
    Variant result;
    result.value_.vt = VT_DISPATCH;
    result.value_.pdispVal = nullptr;
    return result;
}

Variant Variant::doubles(const std::vector<double>& values)
{
    Variant result;
    SAFEARRAYBOUND bound{};
    bound.cElements = static_cast<ULONG>(values.size());
    bound.lLbound = 0;
    result.value_.parray = SafeArrayCreate(VT_R8, 1, &bound);
    if (result.value_.parray == nullptr) throw std::bad_alloc();
    result.value_.vt = VT_ARRAY | VT_R8;
    double* data = nullptr;
    check_hresult(SafeArrayAccessData(result.value_.parray,
                                      reinterpret_cast<void**>(&data)),
                  "SafeArrayAccessData");
    std::copy(values.begin(), values.end(), data);
    check_hresult(SafeArrayUnaccessData(result.value_.parray), "SafeArrayUnaccessData");
    return result;
}

Variant Variant::byref_i4_variant(long initial)
{
    Variant result;
    result.byref_i4_ = std::make_shared<long>(initial);
    result.value_.vt = VT_BYREF | VT_I4;
    result.value_.plVal = result.byref_i4_.get();
    return result;
}

bool Variant::is_nullish() const noexcept
{
    return value_.vt == VT_EMPTY || value_.vt == VT_NULL ||
           (value_.vt == VT_DISPATCH && value_.pdispVal == nullptr);
}

double Variant::as_double() const
{
    if (value_.vt == VT_R8) return value_.dblVal;
    if (value_.vt == VT_R4) return value_.fltVal;
    if (value_.vt == VT_I4) return static_cast<double>(value_.lVal);
    if (value_.vt == VT_I8) return static_cast<double>(value_.llVal);
    if (value_.vt == VT_UI4) return static_cast<double>(value_.ulVal);
    if (value_.vt == VT_BOOL) return value_.boolVal == VARIANT_FALSE ? 0.0 : 1.0;
    throw std::runtime_error("COM value is not numeric");
}

long Variant::as_long() const
{
    if (value_.vt == (VT_BYREF | VT_I4)) return *value_.plVal;
    if (value_.vt == VT_I4) return value_.lVal;
    if (value_.vt == VT_I2) return value_.iVal;
    if (value_.vt == VT_UI4) return static_cast<long>(value_.ulVal);
    return static_cast<long>(as_double());
}

bool Variant::as_bool() const
{
    if (value_.vt == VT_BOOL) return value_.boolVal != VARIANT_FALSE;
    if (value_.vt == VT_I4) return value_.lVal != 0;
    throw std::runtime_error("COM value is not Boolean");
}

std::string Variant::as_string() const
{
    if (value_.vt != VT_BSTR || value_.bstrVal == nullptr) return {};
    return wide_to_utf8(std::wstring_view(value_.bstrVal, SysStringLen(value_.bstrVal)));
}

std::vector<double> Variant::as_doubles() const
{
    if ((value_.vt & VT_ARRAY) == 0 || (value_.vt & VT_TYPEMASK) != VT_R8 ||
        value_.parray == nullptr) {
        throw std::runtime_error("COM value is not a double SAFEARRAY");
    }
    LONG lower = 0;
    LONG upper = -1;
    check_hresult(SafeArrayGetLBound(value_.parray, 1, &lower), "SafeArrayGetLBound");
    check_hresult(SafeArrayGetUBound(value_.parray, 1, &upper), "SafeArrayGetUBound");
    std::vector<double> result(static_cast<std::size_t>(upper - lower + 1));
    if (!result.empty()) {
        double* data = nullptr;
        check_hresult(SafeArrayAccessData(value_.parray,
                                          reinterpret_cast<void**>(&data)),
                      "SafeArrayAccessData");
        std::copy(data, data + result.size(), result.begin());
        check_hresult(SafeArrayUnaccessData(value_.parray), "SafeArrayUnaccessData");
    }
    return result;
}

std::vector<Ptr<IDispatch>> Variant::as_dispatches() const
{
    if ((value_.vt & VT_ARRAY) == 0 || value_.parray == nullptr) {
        throw std::runtime_error("COM value is not a dispatch SAFEARRAY");
    }
    LONG lower = 0;
    LONG upper = -1;
    check_hresult(SafeArrayGetLBound(value_.parray, 1, &lower), "SafeArrayGetLBound");
    check_hresult(SafeArrayGetUBound(value_.parray, 1, &upper), "SafeArrayGetUBound");
    std::vector<Ptr<IDispatch>> result;
    result.reserve(static_cast<std::size_t>(upper - lower + 1));
    if ((value_.vt & VT_TYPEMASK) == VT_DISPATCH) {
        IDispatch** data = nullptr;
        check_hresult(SafeArrayAccessData(value_.parray,
                                          reinterpret_cast<void**>(&data)),
                      "SafeArrayAccessData");
        for (LONG i = lower; i <= upper; ++i) result.emplace_back(data[i - lower]);
        check_hresult(SafeArrayUnaccessData(value_.parray), "SafeArrayUnaccessData");
        return result;
    }
    if ((value_.vt & VT_TYPEMASK) == VT_VARIANT) {
        VARIANT* data = nullptr;
        check_hresult(SafeArrayAccessData(value_.parray,
                                          reinterpret_cast<void**>(&data)),
                      "SafeArrayAccessData");
        for (LONG i = lower; i <= upper; ++i) {
            if (data[i - lower].vt == VT_DISPATCH && data[i - lower].pdispVal != nullptr)
                result.emplace_back(data[i - lower].pdispVal);
        }
        check_hresult(SafeArrayUnaccessData(value_.parray), "SafeArrayUnaccessData");
        return result;
    }
    throw std::runtime_error("COM SAFEARRAY is not dispatch-based");
}

IDispatch* Variant::as_dispatch() const
{
    if (value_.vt != VT_DISPATCH || value_.pdispVal == nullptr)
        throw std::runtime_error("COM value is not an IDispatch");
    return value_.pdispVal;
}

long& Variant::byref_i4_value() const
{
    if (!byref_i4_) throw std::runtime_error("COM value is not an owned VT_BYREF|VT_I4");
    return *byref_i4_;
}

DISPID Dispatch::id(std::string_view name) const
{
    if (!value_) throw std::runtime_error("cannot resolve a member on a null IDispatch");
    const std::wstring wide = utf8_to_wide(name);
    LPOLESTR names[] = {const_cast<LPOLESTR>(wide.c_str())};
    DISPID member = DISPID_UNKNOWN;
    check_hresult(value_->GetIDsOfNames(IID_NULL, names, 1, LOCALE_USER_DEFAULT, &member),
                  "GetIDsOfNames(" + std::string(name) + ")");
    return member;
}

Variant Dispatch::invoke_once(DISPID member, WORD flags,
                              const std::vector<Variant>& arguments,
                              std::string_view operation) const
{
    std::vector<VARIANTARG> native(arguments.size());
    for (auto& argument : native) VariantInit(&argument);
    for (std::size_t i = 0; i < arguments.size(); ++i) {
        check_hresult(VariantCopy(&native[arguments.size() - i - 1],
                                  const_cast<VARIANT*>(&arguments[i].native())),
                      operation_name(operation, "VariantCopy"));
    }
    DISPPARAMS params{};
    params.rgvarg = native.empty() ? nullptr : native.data();
    params.cArgs = static_cast<UINT>(native.size());
    DISPID named = DISPID_PROPERTYPUT;
    if ((flags & DISPATCH_PROPERTYPUT) != 0) {
        params.rgdispidNamedArgs = &named;
        params.cNamedArgs = 1;
    }
    VARIANT result;
    VariantInit(&result);
    EXCEPINFO exception{};
    UINT argument_error = 0;
    const HRESULT status = value_->Invoke(member, IID_NULL, LOCALE_USER_DEFAULT, flags,
                                           &params, &result, &exception, &argument_error);
    for (auto& argument : native) VariantClear(&argument);
    if (FAILED(status)) {
        VariantClear(&result);
        throw Error(status, operation_name(operation, "IDispatch::Invoke"));
    }
    Variant wrapped;
    wrapped.native() = result;
    return wrapped;
}

Variant Dispatch::invoke(std::string_view name, WORD flags,
                         const std::vector<Variant>& arguments,
                         std::string_view operation) const
{
    const auto member = id(name);
    try {
        return invoke_once(member, flags, arguments, operation);
    } catch (const Error& error) {
        if ((flags & DISPATCH_METHOD) == 0 || !can_retry_as_property(error.hresult()))
            throw;
        return invoke_once(member, DISPATCH_PROPERTYGET, arguments, operation);
    }
}

Variant Dispatch::call(std::string_view name, const std::vector<Variant>& arguments,
                       std::string_view operation) const
{
    return invoke(name, DISPATCH_METHOD, arguments,
                  operation.empty() ? std::string(name) : operation);
}

Variant Dispatch::get(std::string_view name, std::string_view operation) const
{
    return invoke(name, DISPATCH_PROPERTYGET, {},
                  operation.empty() ? std::string(name) : operation);
}

void Dispatch::put(std::string_view name, const Variant& value,
                   std::string_view operation) const
{
    invoke(name, DISPATCH_PROPERTYPUT, {value},
           operation.empty() ? std::string(name) : operation);
}

Dispatch Dispatch::call_dispatch(std::string_view name,
                                 const std::vector<Variant>& arguments,
                                 std::string_view operation) const
{
    return Dispatch(call(name, arguments, operation).as_dispatch());
}

Dispatch Dispatch::get_dispatch(std::string_view name, std::string_view operation) const
{
    return Dispatch(get(name, operation).as_dispatch());
}

Dispatch create_dispatch(std::string_view progid)
{
    CLSID clsid{};
    const auto wide = utf8_to_wide(progid);
    check_hresult(CLSIDFromProgID(wide.c_str(), &clsid),
                  "CLSIDFromProgID(" + std::string(progid) + ")");

    Ptr<IUnknown> unknown;
    IUnknown* active = nullptr;
    HRESULT status = GetActiveObject(clsid, nullptr, &active);
    if (SUCCEEDED(status)) unknown = Ptr<IUnknown>(active);
    if (FAILED(status)) {
        check_hresult(CoCreateInstance(clsid, nullptr, CLSCTX_LOCAL_SERVER,
                                       __uuidof(IUnknown),
                                       reinterpret_cast<void**>(&unknown)),
                      "CoCreateInstance(" + std::string(progid) + ")");
    }
    return Dispatch(unknown.query<IDispatch>());
}

} // namespace geargen::solidworks::com

#endif
