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

message(STATUS "========================================")
message(STATUS "[i2w] i2w source dir  : ${I2W_SOURCE_DIR}")
message(STATUS "[crawler-i2w-msgs] crawler-i2w-msgs source dir  : ${I2W_SOURCE_DIR}")
message(STATUS "========================================")

if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "i2w source not found at '${I2W_SOURCE_DIR}'")
endif()
if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "crawler-i2w-msgs source not found at '${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
endif()

add_subdirectory("${I2W_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/i2w")
add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")

set(LOGGER_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../utils/logger"
)
if(NOT EXISTS "${LOGGER_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Logger source not found at '${LOGGER_SOURCE_DIR}'")
endif()    
add_subdirectory("${LOGGER_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/logger")

message(STATUS "========================================")
message(STATUS "[Logger] Logger source dir  : ${LOGGER_SOURCE_DIR}")
message(STATUS "========================================")



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
  logger
)

install(TARGETS robot_mcu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

target_compile_definitions(robot_mcu_node PRIVATE
CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")