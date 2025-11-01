cmake_minimum_required(VERSION 3.5)
project(RX64MotorTest CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the Dynamixel SDK library (parent directory)
get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
set(DXL_SDK_PATH "${PARENT_DIR}/DynamixelSDK/c++")
set(DXL_BUILD_PATH "${PARENT_DIR}/DynamixelSDK/c++/build/mac")

# Include directories
include_directories(
    ${DXL_SDK_PATH}/include
)

# Dynamixel SDK library (built for Mac)
set(DXL_LIBRARY "${DXL_BUILD_PATH}/libdxl_mac_cpp.dylib")

# Check if library exists
if(NOT EXISTS ${DXL_LIBRARY})
    message(WARNING "Dynamixel SDK library not found at ${DXL_LIBRARY}")
    message(STATUS "You may need to build it first:")
    message(STATUS "  cd DynamixelSDK/c++/build/mac && make")
endif()

# Source files
set(DYNAMIXEL_SRCS
    RX64Motor.cpp
    PortFinder.cpp
    main.cpp
)

# Create executable
add_executable(rx64_test ${DYNAMIXEL_SRCS})

# Link libraries
target_link_libraries(rx64_test
    ${DXL_LIBRARY}
)

# Set library search path (RPATH) so it can find the .dylib at runtime
if(APPLE)
    # Set RPATH to find the library
    set_target_properties(rx64_test PROPERTIES
        BUILD_WITH_INSTALL_RPATH FALSE
        INSTALL_RPATH ${DXL_BUILD_PATH}
        BUILD_RPATH ${DXL_BUILD_PATH}
    )
    # Fix the library install name to use @rpath
    add_custom_command(TARGET rx64_test POST_BUILD
        COMMAND install_name_tool -change "./libdxl_mac_cpp.dylib" "@rpath/libdxl_mac_cpp.dylib" $<TARGET_FILE:rx64_test>
        COMMENT "Fixing library path to use @rpath"
    )
endif()

message(STATUS "Dynamixel SDK path: ${DXL_SDK_PATH}")
message(STATUS "Dynamixel library: ${DXL_LIBRARY}")

