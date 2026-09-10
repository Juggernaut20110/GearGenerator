#pragma once

#include <filesystem>
#include <string>

namespace geargen::solidworks {

struct BuildRequest {
    std::string gear_kind;
    std::filesystem::path output_directory;
};

struct BuildResult {
    bool ok{false};
    std::string message;
};

class Backend {
public:
    virtual ~Backend() = default;
    [[nodiscard]] virtual BuildResult build(const BuildRequest& request) = 0;
};

class Session {
public:
    explicit Session(bool visible = true) noexcept : visible_(visible) {}
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // The real implementation will own COM initialization on the calling
    // worker thread. This stage only establishes the lifetime boundary.
    void initialize() noexcept { initialized_ = true; }
    void close() noexcept { initialized_ = false; }
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] bool visible() const noexcept { return visible_; }

private:
    bool visible_{true};
    bool initialized_{false};
};

[[nodiscard]] double millimeters_to_meters(double millimeters) noexcept;

} // namespace geargen::solidworks
