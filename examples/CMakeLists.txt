cmake_minimum_required(VERSION 3.0)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(EXAMPLES

  cartesian_impedance_control
  #[[
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  ]]
)



foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()



#[[
add_executable(leader_TCPIP leader_TCPIP.cpp)
target_link_libraries(leader_TCPIP ncurses Franka::Franka examples_common Eigen3::Eigen3)

add_executable(Yuan_generate_cartesian_pose_motion Yuan_generate_cartesian_pose_motion.cpp)
target_link_libraries(Yuan_generate_cartesian_pose_motion Franka::Franka examples_common Eigen3::Eigen3)

add_executable(Yuan_generate_joint_position_motion Yuan_generate_joint_position_motion.cpp)
target_link_libraries(Yuan_generate_joint_position_motion Franka::Franka examples_common Eigen3::Eigen3)

add_executable(Yuan_generate_joint_velocity_motion Yuan_generate_joint_velocity_motion.cpp)
target_link_libraries(Yuan_generate_joint_velocity_motion Franka::Franka examples_common Eigen3::Eigen3)

add_executable(Yuan_cartesian_impedance_control Yuan_cartesian_impedance_control.cpp)
target_link_libraries(Yuan_cartesian_impedance_control Franka::Franka examples_common Eigen3::Eigen3)

add_executable(Yuan_joint_impedance_control Yuan_joint_impedance_control.cpp)
target_link_libraries(Yuan_joint_impedance_control Franka::Franka examples_common Eigen3::Eigen3 Threads::Threads)

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)
]]


# OpenHaptics
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)
if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}")
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)

# SDL2 (libsdl2-dev)
find_package(SDL2 REQUIRED)
#include_directories(${SDL2_INCLUDE_DIRS})

#SDL2 (2.0.15) plus dependencies (Known issue: ioctl (GFEATURE): Input/output error, when connecting using Bluetooth.
# If other errors happen, please link the old SDL2 version using above lines)
set(SDL2
  /usr/local/lib/libSDL2.a
  pthread
  dl
  rt
  m
)

add_executable(OpenHaptics_try OpenHaptics_try.cpp)
target_link_libraries(OpenHaptics_try HD HDU rt ncurses)

add_executable(HD_example_CoulombField HD_example_CoulombField.cpp helper.cpp helper.h)
target_link_libraries(HD_example_CoulombField HD HDU rt ncurses GL GLU glut)

add_executable(HD_Franka_Simple_Motion_Mirror HD_Franka_Simple_Motion_Mirror.cpp)
#target_link_libraries(HD_Franka_Simple_Motion_Mirror HD HDU rt ncurses GL GLU glut Franka::Franka examples_common Eigen3::Eigen3 Threads::Threads ${SDL2_LIBRARIES})
target_link_libraries(HD_Franka_Simple_Motion_Mirror HD HDU rt ncurses GL GLU glut Franka::Franka examples_common Eigen3::Eigen3 Threads::Threads ${SDL2})

add_executable(SDL_Attempt SDL_Attempt.cpp)
#target_link_libraries(SDL_Attempt ${SDL2_LIBRARIES})
target_link_libraries(SDL_Attempt ${SDL2})


add_executable(test_pinv_time test_pinv_time.cpp)
target_link_libraries(test_pinv_time Eigen3::Eigen3 Threads::Threads)


include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
