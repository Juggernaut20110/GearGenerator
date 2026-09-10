cmake_minimum_required(VERSION 3.21)

project(GearGenerator
    VERSION 0.1.0
    DESCRIPTION "Native C++ port foundation for Gear Generator"
    LANGUAGES CXX
)

option(GEARGEN_BUILD_GUI "Build the Qt Widgets application when Qt is available" ON)
option(GEARGEN_BUILD_TESTS "Build the dependency-free native smoke tests" ON)
option(GEARGEN_BUILD_REFERENCE_TESTS "Build Python-reference regression tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GearGenWarnings)

add_library(geargen_core STATIC
    src/core/common/geometry_types.cpp
    src/core/common/parameters.cpp
    src/core/common/serialization.cpp
    src/core/involute/involute.cpp
    src/core/placement/placement.cpp
    src/core/validation/validation.cpp
    src/core/spur/spur.cpp
    src/core/spur/mesh.cpp
    src/core/bevel/bevel.cpp
    src/core/bevel/mesh.cpp
    src/core/hypoid/hypoid.cpp
    src/core/hypoid/mesh.cpp
    src/core/planetary/planetary.cpp
    src/core/planetary/mesh.cpp
)
target_include_directories(geargen_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(geargen_core PUBLIC cxx_std_20)
geargen_enable_warnings(geargen_core)

add_library(geargen_preview STATIC
    src/preview/scene2d/scene.cpp
    src/preview/scene2d/spur_scene.cpp
    src/preview/scene2d/bevel_scene.cpp
    src/preview/scene2d/hypoid_scene.cpp
    src/preview/scene2d/planetary_scene.cpp
    src/preview/scene3d/scene.cpp
    src/preview/scene3d/assembly.cpp
    src/preview/export/export.cpp
)
target_link_libraries(geargen_preview PUBLIC geargen_core)
target_include_directories(geargen_preview
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(geargen_preview PUBLIC cxx_std_20)
geargen_enable_warnings(geargen_preview)

add_library(geargen_solidworks STATIC
    src/solidworks/solidworks.cpp
)
target_link_libraries(geargen_solidworks PUBLIC geargen_core)
target_include_directories(geargen_solidworks
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(geargen_solidworks PUBLIC cxx_std_20)
geargen_enable_warnings(geargen_solidworks)

if(GEARGEN_BUILD_GUI)
    # Qt is deliberately discovered here rather than referenced by core or
    # preview. This keeps the native math/test targets buildable on machines
    # that do not have a desktop Qt installation yet.
    find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets QUIET)
    if(QT_FOUND)
        find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

        add_library(geargen_gui STATIC
            src/gui/main_window.cpp
        )
        target_link_libraries(geargen_gui
            PUBLIC
                geargen_core
                geargen_preview
                geargen_solidworks
                Qt${QT_VERSION_MAJOR}::Widgets
        )
        target_include_directories(geargen_gui
            PUBLIC
                "${CMAKE_CURRENT_SOURCE_DIR}/src"
        )
        target_compile_features(geargen_gui PUBLIC cxx_std_20)
        geargen_enable_warnings(geargen_gui)

        add_executable(GearGenerator src/main.cpp)
        target_link_libraries(GearGenerator PRIVATE geargen_gui)
        target_compile_features(GearGenerator PRIVATE cxx_std_20)
        geargen_enable_warnings(GearGenerator)
    else()
        message(WARNING
            "Qt Widgets was not found; GUI targets are omitted. "
            "Install Qt 5/6 and configure again, or set "
            "-DGEARGEN_BUILD_GUI=OFF for a core/preview build.")
    endif()
endif()

if(GEARGEN_BUILD_TESTS)
    enable_testing()
    add_executable(geargen_tests
        tests/core_tests.cpp
    )
    target_link_libraries(geargen_tests
        PRIVATE
            geargen_core
            geargen_preview
            geargen_solidworks
    )
    target_compile_features(geargen_tests PRIVATE cxx_std_20)
    geargen_enable_warnings(geargen_tests)
    add_test(NAME geargen_tests COMMAND geargen_tests)

    if(GEARGEN_BUILD_REFERENCE_TESTS)
        file(TO_CMAKE_PATH
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/python_reference.json"
             GEARGEN_REFERENCE_FIXTURE_PATH)
        add_executable(geargen_reference_tests
            tests/reference_regression.cpp
        )
        target_link_libraries(geargen_reference_tests PRIVATE geargen_core)
        target_compile_definitions(geargen_reference_tests PRIVATE
            GEARGEN_REFERENCE_FIXTURE_PATH="${GEARGEN_REFERENCE_FIXTURE_PATH}"
        )
        target_compile_features(geargen_reference_tests PRIVATE cxx_std_20)
        geargen_enable_warnings(geargen_reference_tests)
        add_test(NAME geargen_reference_tests COMMAND geargen_reference_tests)
    endif()
endif()
