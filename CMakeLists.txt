cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(FindPkgConfig)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

pkg_check_modules(NCURSES ncursesw)

if(NCURSES_FOUND)
  include_directories(${NCURSES_INCLUDE_DIRS})
  link_libraries(${NCURSES_LIBRARIES})
else()
  message(FATAL_ERROR "ncurses not found on system.")
endif()

rosbuild_add_executable(nodemon_tui src/main.cpp src/nodemon_tui.cpp)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(nodemon_tui thread)
