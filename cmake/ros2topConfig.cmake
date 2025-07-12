# ros2topConfig.cmake
# CMake configuration file for ros2top

# Find the installation prefix
get_filename_component(ros2top_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(ros2top_CMAKE_PREFIX "${ros2top_CMAKE_DIR}/../.." ABSOLUTE)

# Set include directory - check multiple possible locations
set(ros2top_POSSIBLE_INCLUDE_DIRS 
    "${ros2top_CMAKE_PREFIX}/include"
    "${ros2top_CMAKE_PREFIX}/../include"
    "${ros2top_CMAKE_PREFIX}/../../include"
)

# Find the actual include directory
foreach(possible_dir ${ros2top_POSSIBLE_INCLUDE_DIRS})
    if(EXISTS "${possible_dir}/ros2top/ros2top.hpp")
        set(ros2top_INCLUDE_DIRS "${possible_dir}")
        break()
    endif()
endforeach()

# Check if the header exists
if(ros2top_INCLUDE_DIRS AND EXISTS "${ros2top_INCLUDE_DIRS}/ros2top/ros2top.hpp")
    set(ros2top_FOUND TRUE)
    
    # Create imported target
    if(NOT TARGET ros2top::ros2top)
        add_library(ros2top::ros2top INTERFACE IMPORTED)
        target_include_directories(ros2top::ros2top INTERFACE "${ros2top_INCLUDE_DIRS}")
        
        # Add C++17 requirement (needed for std::filesystem)
        target_compile_features(ros2top::ros2top INTERFACE cxx_std_17)
        
        # Link filesystem library if needed (for older compilers)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
            target_link_libraries(ros2top::ros2top INTERFACE stdc++fs)
        endif()
    endif()
    
    message(STATUS "Found ros2top: ${ros2top_INCLUDE_DIRS}")
    
    # Set variables for compatibility
    set(ros2top_LIBRARIES ros2top::ros2top)
    
else()
    set(ros2top_FOUND FALSE)
    message(WARNING "ros2top headers not found. Checked paths: ${ros2top_POSSIBLE_INCLUDE_DIRS}")
endif()

# Provide helper macro for easy integration
macro(ros2top_target_link target_name)
    if(ros2top_FOUND)
        target_link_libraries(${target_name} ros2top::ros2top)
    else()
        message(WARNING "ros2top not found, cannot link to target ${target_name}")
    endif()
endmacro()
