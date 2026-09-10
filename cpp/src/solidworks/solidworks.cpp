#include "solidworks/solidworks.hpp"

#include "core/bevel/bevel.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "solidworks/assembly_builder.hpp"

#include <type_traits>
#include <utility>

#ifdef _WIN32
#include "solidworks/com.hpp"

#include <utility>
#endif

namespace geargen::solidworks {

struct Session::Impl {
#ifdef _WIN32
    std::unique_ptr<com::Apartment> apartment;
    com::Dispatch application;
#endif
};

Session::Session(bool visible, bool silent)
    : impl_(std::make_unique<Impl>()), visible_(visible), silent_(silent)
{
}

Session::Session(Session&& other) noexcept
    : impl_(std::move(other.impl_)), visible_(other.visible_), silent_(other.silent_),
      initialized_(other.initialized_)
{
    other.initialized_ = false;
}

Session& Session::operator=(Session&& other) noexcept
{
    if (this == &other) return *this;
    close();
    impl_ = std::move(other.impl_);
    visible_ = other.visible_;
    silent_ = other.silent_;
    initialized_ = other.initialized_;
    other.initialized_ = false;
    return *this;
}

Session::~Session()
{
    close();
}

void Session::initialize()
{
    if (initialized_) return;
#ifdef _WIN32
    try {
        impl_->apartment = std::make_unique<com::Apartment>();
        impl_->application = com::create_dispatch("SldWorks.Application");
        if (!impl_->application) throw SwError("could not create SldWorks.Application");
        impl_->application.put("Visible", com::Variant(visible_), "set SOLIDWORKS visibility");
        if (silent_)
            impl_->application.put("CommandInProgress", com::Variant(true),
                                   "enable SOLIDWORKS silent command mode");
        initialized_ = true;
    } catch (const std::exception& error) {
        close();
        throw SwError(std::string("SOLIDWORKS session initialization failed: ") + error.what());
    }
#else
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

void Session::close() noexcept
{
#ifdef _WIN32
    if (impl_ != nullptr) {
        try {
            if (impl_->application && silent_)
                impl_->application.put("CommandInProgress", com::Variant(false),
                                       "restore SOLIDWORKS command mode");
        } catch (...) {
        }
        impl_->application = {};
        impl_->apartment.reset();
    }
#endif
    initialized_ = false;
}

com::Dispatch Session::application() const
{
    if (!initialized_) throw SwError("SOLIDWORKS session is not initialized");
#ifdef _WIN32
    return impl_->application;
#else
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

com::Dispatch Session::new_part() const
{
    const auto app = application();
#ifdef _WIN32
    const auto template_path = app.call(
        "GetUserPreferenceStringValue",
        {com::Variant(static_cast<long>(UserPreferenceString::DefaultTemplatePart))},
        "read default part template").as_string();
    if (template_path.empty())
        throw SwError("no default part template is configured; set one under Tools > Options > File Locations");
    const auto model = app.call_dispatch("NewDocument",
                                         {com::Variant(template_path), com::Variant(0),
                                          com::Variant(0.0), com::Variant(0.0)},
                                         "new part");
    return model;
#else
    (void)app;
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

com::Dispatch Session::new_assembly() const
{
    const auto app = application();
#ifdef _WIN32
    const auto template_path = app.call(
        "GetUserPreferenceStringValue",
        {com::Variant(static_cast<long>(UserPreferenceString::DefaultTemplateAssembly))},
        "read default assembly template").as_string();
    if (template_path.empty())
        throw SwError("no default assembly template is configured; set one under Tools > Options > File Locations");
    return app.call_dispatch("NewDocument",
                             {com::Variant(template_path), com::Variant(0),
                              com::Variant(0.0), com::Variant(0.0)},
                             "new assembly");
#else
    (void)app;
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

com::Dispatch Session::math_utility() const
{
    const auto app = application();
#ifdef _WIN32
    return app.call_dispatch("GetMathUtility", {}, "GetMathUtility");
#else
    (void)app;
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

void Session::save(const com::Dispatch& model, const std::filesystem::path& path) const
{
#ifdef _WIN32
    const auto extension = model.get_dispatch("Extension", "read model Extension");
    auto errors = com::Variant::byref_i4_variant();
    auto warnings = com::Variant::byref_i4_variant();
    const auto ok = extension.call(
        "SaveAs3",
        {com::Variant(path.string()), com::Variant(static_cast<long>(SaveAsVersion::Current)),
         com::Variant(static_cast<long>(SaveAsOptions::Silent)), com::Variant::null_dispatch(),
         com::Variant::null_dispatch(), errors, warnings},
        "SaveAs3");
    if (ok.is_nullish() || !ok.as_bool()) {
        throw SwError("SaveAs3 failed for " + path.string() + " (error " +
                      std::to_string(errors.byref_i4_value()) + ", warning " +
                      std::to_string(warnings.byref_i4_value()) + ")");
    }
#else
    (void)model;
    (void)path;
    throw SwError("native SOLIDWORKS automation is only available on Windows");
#endif
}

void Session::close_document(const com::Dispatch& model,
                             const std::filesystem::path* save_as) const
{
    if (save_as != nullptr) save(model, *save_as);
#ifdef _WIN32
    const auto title = application();
    title.call("CloseDoc", {com::Variant(model.call("GetTitle", {}, "read document title").as_string())},
               "close SOLIDWORKS document");
#else
    (void)model;
#endif
}

BuildResult NativeBackend::build(const BuildRequest& request)
{
    return solidworks::build(request);
}

BuildResult build(const BuildRequest& request)
{
#ifdef _WIN32
    try {
        Session session(request.options.visible, request.options.silent);
        session.initialize();
        return std::visit(
            [&](const auto& parameters) -> BuildResult {
                using T = std::decay_t<decltype(parameters)>;
                if constexpr (std::is_same_v<T, core::BevelSetParams>)
                    return detail::build_bevel_set(session, core::bevel::derive(parameters),
                                                   request.output_directory, request.options);
                else if constexpr (std::is_same_v<T, core::SpurSetParams>)
                    return detail::build_spur_set(session, core::spur::derive(parameters),
                                                  request.output_directory, request.options);
                else if constexpr (std::is_same_v<T, core::HypoidSetParams>)
                    return detail::build_hypoid_set(session, core::hypoid::derive(parameters),
                                                    request.output_directory, request.options);
                else
                    return detail::build_planetary_set(session, core::planetary::derive(parameters),
                                                       request.output_directory, request.options);
            },
            request.parameters);
    } catch (const std::exception& error) {
        BuildResult result;
        result.message = error.what();
        return result;
    }
#else
    (void)request;
    BuildResult result;
    result.message = "native SOLIDWORKS automation is only available on Windows";
    return result;
#endif
}

double millimeters_to_meters(double millimeters) noexcept
{
    return millimeters * 0.001;
}

} // namespace geargen::solidworks
