cmake_minimum_required(VERSION 3.10.2)
project(test_script_omplapp)
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++14.
# Set warning flags by default during development.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wuninitialized -Wpedantic")

add_compile_options(-Wall -Wextra -Wpedantic)
if(TEST_SCRIPT_OMPLAPP_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Link Libraries
#================================================================================
find_package(catkin REQUIRED COMPONENTS)
find_package(DART REQUIRED 
  COMPONENTS utils utils-urdf optimizer-nlopt)
find_package(ompl REQUIRED)
find_package(aikido 0.4.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    planner
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kunzretimer
    robot
    statespace
    io
)

# Install the data
catkin_package()
install(DIRECTORY "${PROJECT_SOURCE_DIR}/ompl_data/"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/ompl_data")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${aikido_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/PlaceHolder.cpp
)

#================================================================================
# Example Scripts
#================================================================================
add_executable(piano examples/piano.cpp)
target_link_libraries(piano PRIVATE ${PROJECT_NAME} ${DART_LIBRARIES} ${aikido_LIBRARIES})
install(TARGETS piano RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#================================================================================
# Installation
#================================================================================

install(TARGETS test_script_omplapp EXPORT test_script_omplappConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT test_script_omplappConfig DESTINATION share/test_script_omplapp/cmake)

# This makes the project importable from the build directory
export(TARGETS test_script_omplapp FILE test_script_omplappConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/test_script_omplapp")

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
