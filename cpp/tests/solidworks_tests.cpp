#include "solidworks/com.hpp"
#include "solidworks/solidworks.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool check(bool condition, const std::string& message)
{
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

} // namespace

#ifdef _WIN32

class RecordingDispatch final : public IDispatch {
public:
    ULONG ref_count{1};
    std::vector<VARTYPE> argument_types;
    std::vector<double> numeric_arguments;

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, void** result) override
    {
        if (result == nullptr) return E_POINTER;
        *result = nullptr;
        if (IsEqualIID(iid, IID_IUnknown) || IsEqualIID(iid, IID_IDispatch)) {
            *result = static_cast<IDispatch*>(this);
            AddRef();
            return S_OK;
        }
        return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return ++ref_count; }
    ULONG STDMETHODCALLTYPE Release() override
    {
        const ULONG value = --ref_count;
        if (value == 0) delete this;
        return value;
    }

    HRESULT STDMETHODCALLTYPE GetTypeInfoCount(UINT* count) override
    {
        if (count == nullptr) return E_POINTER;
        *count = 0;
        return S_OK;
    }

    HRESULT STDMETHODCALLTYPE GetTypeInfo(UINT, LCID, ITypeInfo**) override
    {
        return E_NOTIMPL;
    }

    HRESULT STDMETHODCALLTYPE GetIDsOfNames(REFIID, LPOLESTR* names, UINT count,
                                            LCID, DISPID* ids) override
    {
        if (names == nullptr || ids == nullptr || count != 1) return E_INVALIDARG;
        if (std::wstring(names[0]) != L"RecordedCall") return DISP_E_UNKNOWNNAME;
        ids[0] = 42;
        return S_OK;
    }

    HRESULT STDMETHODCALLTYPE Invoke(DISPID id, REFIID, LCID, WORD flags,
                                     DISPPARAMS* parameters, VARIANT* result,
                                     EXCEPINFO*, UINT*) override
    {
        if (id != 42 || (flags & DISPATCH_METHOD) == 0 || parameters == nullptr)
            return DISP_E_MEMBERNOTFOUND;
        argument_types.clear();
        numeric_arguments.clear();
        for (UINT i = 0; i < parameters->cArgs; ++i) {
            argument_types.push_back(parameters->rgvarg[i].vt);
            if (parameters->rgvarg[i].vt == VT_R8)
                numeric_arguments.push_back(parameters->rgvarg[i].dblVal);
        }
        if (result != nullptr) {
            VariantInit(result);
            result->vt = VT_R8;
            result->dblVal = 7.5;
        }
        return S_OK;
    }
};

#endif

int main()
{
#ifdef _WIN32
    using geargen::solidworks::com::Variant;
    bool ok = true;

    geargen::solidworks::com::Apartment apartment;

    const auto array = Variant::doubles({1.25, -2.5, 3.75});
    const auto values = array.as_doubles();
    ok &= check(values.size() == 3 && std::abs(values[1] + 2.5) < 1e-12,
                "SAFEARRAY double marshalling round-trips values");
    ok &= check(array.type() == (VT_ARRAY | VT_R8),
                "transform arrays use VT_ARRAY|VT_R8");

    auto out_status = Variant::byref_i4_variant();
    out_status.byref_i4_value() = 17;
    const auto copied_status = out_status;
    ok &= check(copied_status.byref_i4_value() == 17,
                "by-reference HRESULT/status outputs retain shared storage on copy");

    const Variant text("native COM boundary");
    ok &= check(text.as_string() == "native COM boundary",
                "BSTR text marshalling round-trips UTF-8 input");

    auto* recorder = new RecordingDispatch();
    geargen::solidworks::com::Dispatch recorded(recorder);
    const auto call_result = recorded.call(
        "RecordedCall", {Variant(1.0), Variant(2.0)}, "record a mock COM call");
    ok &= check(std::abs(call_result.as_double() - 7.5) < 1e-12 &&
                    recorder->argument_types.size() == 2 &&
                    recorder->numeric_arguments.size() == 2 &&
                    recorder->numeric_arguments[0] == 2.0 &&
                    recorder->numeric_arguments[1] == 1.0,
                "IDispatch invocation records reversed COM argument order");
    recorded = {};

    ok &= check(geargen::solidworks::millimeters_to_meters(12.5) == 0.0125,
                "SOLIDWORKS length boundary converts millimetres to metres");
    return ok ? 0 : 1;
#else
    return 0;
#endif
}
