# ros2topConfig.cmake
# CMake package configuration for ros2top's header-only C++ API.
#
# ros2top is distributed as a Python wheel, so this file and the header it
# points at are installed as data files. Where they land depends on how pip was
# invoked: a system install puts them under /usr/local, `pip install --user`
# under ~/.local, and a venv inside that venv. CMake only searches a few system
# prefixes, so the last two are not found automatically -- ask ros2top where it
# put them:
#
#   colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)
#
# A source checkout installs no data files at all, and then this file is used
# straight out of the repository. Both layouts are handled below.

get_filename_component(ros2top_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(ros2top_CMAKE_PREFIX "${ros2top_CMAKE_DIR}/../.." ABSOLUTE)

# Candidate include directories, most likely first.
set(ros2top_POSSIBLE_INCLUDE_DIRS
    # Installed layout: <prefix>/share/ros2top/cmake -> <prefix>/include
    "${ros2top_CMAKE_PREFIX}/../include"
    "${ros2top_CMAKE_PREFIX}/include"
    "${ros2top_CMAKE_PREFIX}/../../include"
    # Source checkout: <repo>/cmake -> <repo>/include
    "${ros2top_CMAKE_DIR}/../include"
)

unset(ros2top_INCLUDE_DIRS)
foreach(possible_dir ${ros2top_POSSIBLE_INCLUDE_DIRS})
    if(EXISTS "${possible_dir}/ros2top/ros2top.hpp")
        get_filename_component(ros2top_INCLUDE_DIRS "${possible_dir}" ABSOLUTE)
        break()
    endif()
endforeach()

if(ros2top_INCLUDE_DIRS)
    set(ros2top_FOUND TRUE)

    if(NOT TARGET ros2top::ros2top)
        add_library(ros2top::ros2top INTERFACE IMPORTED)
        target_include_directories(ros2top::ros2top INTERFACE "${ros2top_INCLUDE_DIRS}")

        # std::filesystem
        target_compile_features(ros2top::ros2top INTERFACE cxx_std_17)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
            target_link_libraries(ros2top::ros2top INTERFACE stdc++fs)
        endif()

        # The header stores registrations as JSON. Link the dependency into the
        # imported target rather than leaving every consumer to remember it --
        # forgetting produced a "nlohmann/json.hpp: No such file" that looked
        # like a ros2top problem.
        find_package(nlohmann_json QUIET)
        if(nlohmann_json_FOUND)
            target_link_libraries(ros2top::ros2top INTERFACE nlohmann_json::nlohmann_json)
        endif()
    endif()

    set(ros2top_LIBRARIES ros2top::ros2top)
    message(STATUS "Found ros2top: ${ros2top_INCLUDE_DIRS}")

    if(NOT nlohmann_json_FOUND)
        message(WARNING
            "ros2top: nlohmann_json was not found. ros2top/ros2top.hpp needs it.\n"
            "  Install it with: sudo apt install nlohmann-json3-dev")
    endif()
else()
    set(ros2top_FOUND FALSE)
    set(ros2top_NOT_FOUND_MESSAGE
        "Found ros2topConfig.cmake at ${ros2top_CMAKE_DIR}, but not the header "
        "ros2top/ros2top.hpp next to it. Checked: "
        "${ros2top_POSSIBLE_INCLUDE_DIRS}. Reinstall with `pip install ros2top`, "
        "then pass -Dros2top_DIR=$(ros2top --cmake-dir).")
    message(WARNING "${ros2top_NOT_FOUND_MESSAGE}")
endif()

# Convenience wrapper for consumers that would rather not name the target.
macro(ros2top_target_link target_name)
    if(ros2top_FOUND)
        target_link_libraries(${target_name} ros2top::ros2top)
    else()
        message(WARNING "ros2top not found, cannot link to target ${target_name}")
    endif()
endmacro()
