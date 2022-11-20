cmake_minimum_required(VERSION 3.17)

project(nvjpeg_compressed_image_transport)

include(CheckLanguage)
check_language(CUDA)
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "cuda not found")
  return()
endif()
	
enable_language(CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(CUDAToolkit)
if(NOT TARGET CUDA::nvjpeg)
  message(WARNING "nvjpeg not found")
  return()
endif()

add_library(${PROJECT_NAME} SHARED
  src/manifest.cpp
  src/nvjpeg_compressed_publisher.cpp
  src/rgb8_to_bgr8.cu
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:-Wno-pedantic>
)

target_link_libraries(${PROJECT_NAME}
  CUDA::cudart
  CUDA::nvjpeg
)

ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)
pluginlib_export_plugin_description_file(image_transport nvjpeg_compressed_plugins.xml)

ament_package()
