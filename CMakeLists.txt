cmake_minimum_required(VERSION 3.22)
set(PROJECT_VERSION 2.3.0)
project(servo_robot_board_bridge VERSION ${PROJECT_VERSION})

option(BUILD_EXE "Build node executable file" ON)
option(BUILD_TESTING ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT SKIP_CXX23_STDLIB_CHECK AND NOT _CXX23_STDLIB_VERIFIED)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_QUIET TRUE)

    string(TOUPPER "${PROJECT_NAME}" _PRJ_UPPER)

    check_cxx_source_compiles([[
        #include <expected>
        #include <format>
        #include <ranges>
        int main() {
            std::expected<int, int> result{42};
            const auto text = std::format("{}", result.value());
            const auto values = std::views::iota(0, 1);
            return static_cast<int>(text.size() + *values.begin());
        }
    ]] ${_PRJ_UPPER}_HAS_REQUIRED_CXX23_STDLIB)

    if (NOT ${_PRJ_UPPER}_HAS_REQUIRED_CXX23_STDLIB)
        message(FATAL_ERROR
                "${PROJECT_NAME} requires a C++23 standard library with <expected>, "
                "<format>, and ranges support. Use GCC 13+ or an equivalent compiler.")
    endif ()
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(PROJECT_EXE_NAME ${PROJECT_NAME}_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if (NOT ROS2_WS_SOURCE_BUILD)
    find_package(servo_robot_board_interface REQUIRED)
    find_package(common_cpp REQUIRED)
endif ()

# ── servo_robot_driver (Rust FFI shared library) ──────────────────────
add_library(servo_robot_driver SHARED IMPORTED GLOBAL)
set_target_properties(servo_robot_driver PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libservo_robot_driver.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include/servo_robot_board_bridge"
)

add_library(${PROJECT_NAME} STATIC
        src/bridge.cpp
        src/lifecycle_node.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        lifecycle_msgs
        diagnostic_msgs
        std_msgs
        sensor_msgs
)
target_link_libraries(${PROJECT_NAME} servo_robot_driver)

if (ROS2_WS_SOURCE_BUILD)
    if (NOT TARGET common_cpp_logging_interface)
        message(FATAL_ERROR
                "Source workspace configuration requires in-tree "
                "common_cpp_logging_interface targets before ${PROJECT_NAME}.")
    endif ()
    target_link_libraries(${PROJECT_NAME}
            common_cpp_logging_interface
    )
else ()
    target_link_libraries(${PROJECT_NAME}
            common_cpp::common_cpp_logging_interface
    )
endif ()

if (ROS2_WS_SOURCE_BUILD AND TARGET servo_robot_board_interface)
    rosidl_get_typesupport_target(servo_robot_board_interface_CPP_TYPESUPPORT_TARGET
            servo_robot_board_interface
            "rosidl_typesupport_cpp"
    )
    rosidl_get_typesupport_target(servo_robot_board_interface_FASTRTPS_CPP_TYPESUPPORT_TARGET
            servo_robot_board_interface
            "rosidl_typesupport_fastrtps_cpp"
    )
    rosidl_get_typesupport_target(servo_robot_board_interface_INTROSPECTION_CPP_TYPESUPPORT_TARGET
            servo_robot_board_interface
            "rosidl_typesupport_introspection_cpp"
    )
    if (servo_robot_board_interface_CPP_TYPESUPPORT_TARGET MATCHES "-NOTFOUND$"
            OR servo_robot_board_interface_FASTRTPS_CPP_TYPESUPPORT_TARGET MATCHES "-NOTFOUND$"
            OR servo_robot_board_interface_INTROSPECTION_CPP_TYPESUPPORT_TARGET MATCHES "-NOTFOUND$")
        message(FATAL_ERROR "The in-tree servo_robot_board_interface target does not provide rosidl_typesupport_cpp.")
    endif ()
    if (NOT DEFINED servo_robot_board_interface_SOURCE_DIR
            OR NOT IS_DIRECTORY "${servo_robot_board_interface_SOURCE_DIR}/include")
        message(FATAL_ERROR "The in-tree servo_robot_board_interface source include directory is unavailable.")
    endif ()
    target_link_libraries(${PROJECT_NAME}
            ${servo_robot_board_interface_CPP_TYPESUPPORT_TARGET}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
            "$<BUILD_INTERFACE:${servo_robot_board_interface_SOURCE_DIR}/include>")
else ()
    ament_target_dependencies(${PROJECT_NAME} servo_robot_board_interface)
endif ()

if (ROS2_WS_SOURCE_BUILD)
    target_link_libraries(${PROJECT_NAME}
            "-Wl,--no-as-needed"
            ${servo_robot_board_interface_FASTRTPS_CPP_TYPESUPPORT_TARGET}
            ${servo_robot_board_interface_INTROSPECTION_CPP_TYPESUPPORT_TARGET}
            "-Wl,--as-needed"
    )
endif ()

if (BUILD_EXE)

    add_executable(${PROJECT_EXE_NAME}
            src/main.cpp
    )
    target_compile_features(${PROJECT_EXE_NAME} PRIVATE cxx_std_23)
    target_link_libraries(${PROJECT_EXE_NAME} ${PROJECT_NAME})
    set_target_properties(${PROJECT_EXE_NAME} PROPERTIES
            BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
            INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )
    ament_target_dependencies(${PROJECT_EXE_NAME}
            rclcpp
            rclcpp_lifecycle
            lifecycle_msgs
    )
    if (ROS2_WS_SOURCE_BUILD)
        if (NOT TARGET common_cpp_ros_logging)
            message(FATAL_ERROR
                    "Source workspace configuration requires the in-tree common_cpp_ros_logging "
                    "target before ${PROJECT_NAME}.")
        endif ()
        target_link_libraries(${PROJECT_EXE_NAME} common_cpp_ros_logging)
    else ()
        target_link_libraries(${PROJECT_EXE_NAME}
                ros_common_cpp::common_cpp_ros_logging
        )
    endif ()

endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME} PRIVATE -std=c++2b)
    if (BUILD_EXE)
        target_compile_options(${PROJECT_EXE_NAME} PRIVATE -std=c++2b)
    endif ()
endif ()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_EXE)
    install(TARGETS ${PROJECT_EXE_NAME}
            DESTINATION lib/${PROJECT_NAME}
    )
endif ()

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY include/
        DESTINATION include
)

ament_export_dependencies(
        rclcpp rclcpp_action rclcpp_lifecycle lifecycle_msgs diagnostic_msgs
        std_msgs sensor_msgs
        servo_robot_board_interface
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
