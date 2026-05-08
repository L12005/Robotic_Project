cmake_minimum_required(VERSION 3.8)
project(hmi_gazebo_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Keep Gazebo on the system protobuf toolchain instead of any Windows / Conda headers
# inherited by the shell environment under WSL.
list(APPEND CMAKE_IGNORE_PREFIX_PATH
  "/mnt/e/Anaconda"
  "/mnt/e/Anaconda/Library"
  "/mnt/e/Anaconda/include"
)

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sim8 REQUIRED)

foreach(_protobuf_target IN ITEMS protobuf::libprotobuf protobuf::libprotobuf-lite)
  if(TARGET ${_protobuf_target})
    get_target_property(_protobuf_includes ${_protobuf_target} INTERFACE_INCLUDE_DIRECTORIES)
    if(_protobuf_includes)
      list(FILTER _protobuf_includes EXCLUDE REGEX ".*/Anaconda/Library/include$")
      set_target_properties(${_protobuf_target} PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_protobuf_includes}"
      )
    endif()
  endif()
endforeach()

add_library(hmi_led_ring_system SHARED
  src/led_ring_system.cpp
)

target_compile_features(hmi_led_ring_system PUBLIC cxx_std_17)
target_include_directories(hmi_led_ring_system PRIVATE
  /opt/ros/jazzy/opt/gz_sim_vendor/include/gz/sim8
  /opt/ros/jazzy/opt/gz_msgs_vendor/include/gz/msgs10
)
target_link_libraries(hmi_led_ring_system
  PRIVATE
    gz-plugin2::register
    gz-msgs10::core
    gz-transport13::core
    gz-sim8::gz-sim8
)

install(TARGETS hmi_led_ring_system
  LIBRARY DESTINATION lib
)

ament_package()
