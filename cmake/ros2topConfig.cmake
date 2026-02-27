# ros2topConfig.cmake
# CMake configuration file for ros2top
# Installed by: pip install ros2top

# Resolve the install prefix from this file's location:
#   <prefix>/share/ros2top/cmake/ros2topConfig.cmake  →  <prefix>
get_filename_component(_ros2top_cmake_dir "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_ros2top_prefix "${_ros2top_cmake_dir}/../../.." ABSOLUTE)

set(_ros2top_include_dir "${_ros2top_prefix}/include")

if(EXISTS "${_ros2top_include_dir}/ros2top/ros2top.hpp")
    set(ros2top_FOUND TRUE)
    set(ros2top_INCLUDE_DIRS "${_ros2top_include_dir}")

    if(NOT TARGET ros2top::ros2top)
        add_library(ros2top::ros2top INTERFACE IMPORTED)
        set_target_properties(ros2top::ros2top PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${_ros2top_include_dir}"
            INTERFACE_COMPILE_FEATURES "cxx_std_17"
        )

        # Older GCC (<9) needs explicit filesystem link
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
            set_property(TARGET ros2top::ros2top APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES "stdc++fs")
        endif()
    endif()

    set(ros2top_LIBRARIES ros2top::ros2top)
    message(STATUS "Found ros2top: ${_ros2top_include_dir}/ros2top/ros2top.hpp")
else()
    set(ros2top_FOUND FALSE)
    message(FATAL_ERROR
        "ros2top header not found at ${_ros2top_include_dir}/ros2top/ros2top.hpp\n"
        "Make sure ros2top is installed: pip install ros2top\n"
        "Then set CMAKE_PREFIX_PATH to the pip install prefix (e.g. ~/.local or your venv).")
endif()

unset(_ros2top_cmake_dir)
unset(_ros2top_prefix)
unset(_ros2top_include_dir)
