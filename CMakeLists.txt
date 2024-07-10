cmake_minimum_required(VERSION 3.11)
project(fcat
    VERSION 0.6.1
    LANGUAGES C CXX
    DESCRIPTION "ROS2 Fastcat Node"
    )

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror)
endif()

# Dependencies
include(FetchContent)
FetchContent_Declare(fastcat
    GIT_REPOSITORY https://github.com/nasa-jpl/fastcat.git
    GIT_TAG v0.13.1
    )
FetchContent_MakeAvailable(fastcat)
FetchContent_Declare(jsd
  GIT_REPOSITORY https://github.com/nasa-jpl/jsd.git
  GIT_TAG v3.0.1
)
FetchContent_GetProperties(jsd)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fcat_msgs REQUIRED)

set(soem_SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/soem-src)
set(SOEM_INCLUDE_DIRS
    ${soem_SOURCE_DIR}/soem
    ${soem_SOURCE_DIR}/osal
)
if(WIN32)
  set(OS "win32")
elseif(UNIX AND NOT APPLE)
  set(OS "linux")
elseif(APPLE)
  set(OS "macosx")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  set(OS "rtk")
  list(APPEND SOEM_INCLUDE_DIRS ${soem_SOURCE_DIR}/oshw/${OS}/${ARCH})
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
  set(OS "rtems")
endif()

list(APPEND SOEM_INCLUDE_DIRS
  ${soem_SOURCE_DIR}/osal/${OS}
  ${soem_SOURCE_DIR}/oshw/${OS}
)
# Copies headers to install space 
file(GLOB_RECURSE includes RELATIVE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    ${soem_SOURCE_DIR}
    ${soem_SOURCE_DIR}/*.h
    ${soem_SOURCE_DIR}/*.hh
    ${soem_SOURCE_DIR}/*.hpp
    )
foreach(include ${includes})
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${include}"
        "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${include}"
        "${soem_SOURCE_DIR}/${include}"
        "${CMAKE_BINARY_DIR}/include/${include}"
        COPYONLY
        )
endforeach()

# Create fcat node
# add_executable(fcat
#     src/fcat_main.cpp
#     src/fcat.cpp
#     src/fcat_callbacks.cpp
#     )

add_library(${PROJECT_NAME} SHARED
    src/fcat.cpp
    src/fcat_callbacks.cpp
    src/fcat_utils.cpp
    )

ament_target_dependencies(${PROJECT_NAME} PUBLIC 
    rclcpp
    sensor_msgs
    geometry_msgs
    std_srvs
    fcat_msgs
    rclcpp_components
    )
target_link_libraries(${PROJECT_NAME} PRIVATE
    fastcat
    )

target_compile_definitions(${PROJECT_NAME} PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

add_executable(fcat_node
  src/fcat_main.cpp
)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "humble")
    target_compile_definitions(fcat_node
      PUBLIC
      ROS2_HUMBLE
    )
  endif()
endif()
target_include_directories(
  fcat_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
foreach(include_dir ${SOEM_INCLUDE_DIRS})
  file(RELATIVE_PATH rel ${soem_SOURCE_DIR} ${include_dir})
  message(STATUS ${rel})
  target_include_directories(
    fcat_node PUBLIC
    "$<BUILD_INTERFACE:${include_dir}>"
    "$<INSTALL_INTERFACE:include/${rel}>"
  )
  ament_export_include_directories(include/${rel})
endforeach()

target_link_libraries(
  fcat_node PRIVATE
  ${PROJECT_NAME}
)
add_executable(fcat_services
  src/fcat_services_main.cpp
  src/fcat_services.cpp
  src/fcat_utils.cpp
)
ament_target_dependencies(fcat_services PUBLIC
  rclcpp
  geometry_msgs
  fcat_msgs
  std_srvs
)
target_link_libraries(fcat_services PRIVATE
  fastcat
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_fcat
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY
  ${CMAKE_BINARY_DIR}/include/
  DESTINATION
  include
)
ament_export_include_directories(include)
install(TARGETS
  fcat_node
  fcat_services
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
rclcpp_components_register_nodes(${PROJECT_NAME} "fcat::FcatNode")

ament_package()


