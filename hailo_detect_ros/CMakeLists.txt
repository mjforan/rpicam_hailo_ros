cmake_minimum_required(VERSION 3.16)
project(hailo_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(src/ByteTrack-cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(Threads)
find_package(HailoRT REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB SOURCES
    ./src/Hailo-Application-Code-Examples/runtime/hailo-8/cpp/common/*.cpp
    ./src/Hailo-Application-Code-Examples/runtime/hailo-8/cpp/object_detection/*.cpp
    ./src/Hailo-Application-Code-Examples/runtime/hailo-8/cpp/object_detection/utils/*.cpp
)
add_executable(${PROJECT_NAME} ${SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(${PROJECT_NAME} rclcpp::rclcpp ${vision_msgs_TARGETS})
target_link_libraries(${PROJECT_NAME} Threads::Threads HailoRT::libhailort)
target_link_libraries(${PROJECT_NAME} bytetrack)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Hailo-Application-Code-Examples/runtime/hailo-8/cpp/common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Hailo-Application-Code-Examples/runtime/hailo-8/cpp/object_detection/utils>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
  ${bytetrack_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
