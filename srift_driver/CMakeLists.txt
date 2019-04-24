cmake_minimum_required(VERSION 2.8.3)
project(srift_driver)


find_package(catkin REQUIRED COMPONENTS genmsg roscpp diagnostic_updater std_msgs geometry_msgs message_generation tf  cmake_modules)
find_package(Boost)
find_package(Eigen REQUIRED)

add_service_files(FILES
    SRI_String_cmd.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs diagnostic_updater diagnostic_msgs message_runtime tf
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/srift_driver/srift_driver.h
                 ${INCLUDE_DIR}/srift_driver/srift_bias.h
                 ${INCLUDE_DIR}/srift_driver/ft_listener.h
                 ${INCLUDE_DIR}/srift_driver/TCPClient.h
                 ${INCLUDE_DIR}/srift_driver/M812x_def.h
)

set(SRC_FILES   ${SRC_DIR}/srift_driver.cpp
                ${SRC_DIR}/srift_bias.cpp
                ${SRC_DIR}/ft_listener.cpp
                ${SRC_DIR}/TCPClient.cpp
                ${SRC_DIR}/ft_listener.cpp

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${KALMAN_INCLUDE}
  )
include_directories(include)

include_directories(${Eigen_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(srift_node ${SRC_DIR}/srift_node.cpp      )
target_link_libraries(srift_node ${PROJECT_NAME} boost_program_options)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
