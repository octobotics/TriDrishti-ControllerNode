cmake_minimum_required(VERSION 3.22)

project(robot LANGUAGES CXX)

set(I2W_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../i2w"
    CACHE PATH
    "Path to the i2w source tree")
set(CRAWLER_I2W_MSGS_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../crawler-i2w-msgs"
    CACHE PATH
    "Path to the crawler i2w message package")

if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "i2w source not found at '${I2W_SOURCE_DIR}'")
endif()
if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "crawler-i2w-msgs source not found at '${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
endif()

add_subdirectory("${I2W_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/i2w")
add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")

add_executable(robot_mcu_node
  src/main.cpp

  src/robot/robot.cpp

  src/managers/i2wManager.cpp
)
target_compile_features(robot_mcu_node PRIVATE cxx_std_20)
target_include_directories(robot_mcu_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(robot_mcu_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS robot_mcu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")


find_package(logger)
find_package(behaviortree_cpp_v3)


add_executable(robot_node
src/main.cpp
)
target_compile_features(robot_node PRIVATE cxx_std_20)
target_include_directories(robot_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(robot_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
  logger
  behaviortree_cpp_v3
)

install(TARGETS robot_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

