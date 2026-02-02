cmake_minimum_required(VERSION 3.8)
project(rob5_implem)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Find pkg-config
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED )


# Find libgpiod
pkg_check_modules(GPIOD REQUIRED libgpiod)
pkg_check_modules(GPIODPP REQUIRED libgpiodcxx)

# Add include directories
include_directories(${GPIOD_INCLUDE_DIRS})
include_directories(${GPIODPP_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(hardware src/hardware.cpp)
add_executable(magneto src/magneto.cpp)
add_executable(encoder src/encoder.cpp)
add_executable(camera src/camera.cpp)

ament_target_dependencies(hardware rclcpp std_msgs sensor_msgs)
ament_target_dependencies(magneto rclcpp std_msgs sensor_msgs)
ament_target_dependencies(encoder rclcpp std_msgs sensor_msgs)
ament_target_dependencies(camera rclcpp std_msgs sensor_msgs)

# Link the library to your target
target_link_libraries(encoder ${GPIOD_LIBRARIES})
target_link_libraries(encoder ${GPIODPP_LIBRARIES})
target_link_libraries(camera ${OpenCV_LIBS})

foreach(target hardware magneto encoder camera)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)
endforeach()

#install executables
install(TARGETS hardware magneto encoder camera DESTINATION lib/${PROJECT_NAME})
#install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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
