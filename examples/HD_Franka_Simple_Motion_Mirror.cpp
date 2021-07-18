// Copyright (c) 2017 Franka Emika GmbH; Bristol Robotics Laboratory (UK).
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*****************************************************************************
Author: Yuan (Vincent) Guan
Email: yuan2.guan@brl.ac.uk / vincentguan@ieee.org
 For questions, comments or bug reports, please send email or check the Github repo at:
    http:/xxxxxxxxxxxxxxxxxxxxxxxxxxx
File:
  HD_Franks_Simple_Motion_Mirror.cpp
Description:
  xxxxxxxxxxxxxxx
*******************************************************************************/

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <array>
#include <cmath>
#include <functional>
#include <SDL.h>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

/* <Graphics> */
#if defined(WIN32)
# include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

/* <Static> */
static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle hUpdateDeviceCallback = HD_INVALID_HANDLE;
static bool isRunning = true;

/* <Struct> */
typedef struct
{
  hduVector3Dd start_position;
  hduVector3Dd Delta_position;
  HDdouble Robot_transform[16];
  HDdouble HD_transform[16];
  hduVector3Dd anchor;
  HDboolean isAnchorActive;
} HapticState;

/* <Function> */
void exitHandler();
void initHD();
//
HDCallbackCode HDCALLBACK hapticCallback(void *pUserData);

/* <Global> */
//double time_now = 0;
SDL_Event Event;
HapticState state;


/**
 * @Function Check_KeyEvent
 *  Function used to monitor the keyboard events.
 * @in Null
 * @return Null
 */
void Check_KeyEvent ()
{
  while (isRunning) {
    while (SDL_PollEvent(&Event) != 0) {
      if (Event.type == SDL_KEYDOWN) {
        switch (Event.key.keysym.sym) {
          case SDLK_q: isRunning = false; break;
        }
      }
    }
  }
}


/**
 * @Application HD_Franks_Simple_Motion_Mirror.cpp
 *  is an example showing how to teleoperate the Franka Panda Robot using 3DSystem Touch
 *  haptic device.
 *
 * @Warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv)
{
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  std::atexit(exitHandler);
  initHD();
  std::cout << "HDAPI initialised." << std::endl;

  // TODO: raise a flag of state data ready. after that we can do the control.
  std::cout << state.Delta_position << std::endl;

  ////////////////////////// <START> Initiate Stiffness and Damping matrix ////////////////////////
  // Compliance parameters
  const double translational_stiffness{500.0};
  const double rotational_stiffness{50.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);

  // Toggle this to have a higher stiffness in the z-axis.
  //stiffness(2,2) = 10 * stiffness(2,2);

  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                 Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  ////////////////////////// <END> Initiate Stiffness and Damping matrix //////////////////////////

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    // TODO: Usage of Eigen::Map (reuse memory of a variable as Eigen type)
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    ////////////////////////// <START> For sake of Debugging /////////////////////////////
    std::cout << position_d << std::endl;
    std::cout << orientation_d.w() << "-w "
              << orientation_d.x() << "-x "
              << orientation_d.y() << "-y "
              << orientation_d.z() << "-z "
              << std::endl;

    Eigen::Affine3d F2EE_transform(Eigen::Matrix4d::Map(initial_state.F_T_EE.data()));
    Eigen::Vector3d F2EE_position_d(F2EE_transform.translation());
    Eigen::Quaterniond F2EE_orientation_d(F2EE_transform.linear());

    std::cout << F2EE_position_d << std::endl;
    std::cout << F2EE_orientation_d.w() << "-w "
              << F2EE_orientation_d.x() << "-x "
              << F2EE_orientation_d.y() << "-y "
              << F2EE_orientation_d.z() << "-z "
              << std::endl;

    // TODO: Clear this after usage for debugging purpose.
    std::cout << "... Debugging. Please press Enter to continue.";
    std::cin.ignore();
    ////////////////////////// <END> For sake of Debugging /////////////////////////////

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    double time = 0.0;
    std::array<double, 16> initial_pose;

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());


      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      time += duration.toSec();

      //position_d(0) = initial_transform.translation()[0] + 0.0005 * state.Delta_position[0];
      //position_d(1) = initial_transform.translation()[1] + 0.0005 * state.Delta_position[1];
      //position_d(2) = initial_transform.translation()[2] + 0.0005 * state.Delta_position[2];

      if (state.isAnchorActive) {
         transform = Eigen::Matrix4d::Map(state.HD_transform);
         orientation_d = transform.linear();
      }

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      //std::cout << orientation_d.coeffs() << std::endl;
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);


      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * ( -stiffness * error - damping * (jacobian * dq) );
      //tau_task.setZero();
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Toggle this to enter the guiding mode
      //tau_d_array = {{0,0,0,0,0,0,0}};

      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

    ///////////////////////// <START> Creating new thread ///////////////////////
    std::thread Check_KeyEvent_thread(Check_KeyEvent);
    Check_KeyEvent_thread.join();

    while (isRunning) {
      // TODO:wait....
    }
    std::cout << "Please wait, we will quit the program..." << std::endl;
    return 0;
    ////////////////////////// <END> Creating new thread ////////////////////////

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}

/**
 * @Function exitHandler()
 *  The handler get called when the application is exiting to shutdown the HD API.
 */
void exitHandler()
{
  hdStopScheduler();
  hdUnschedule(hUpdateDeviceCallback);
  if (ghHD != HD_INVALID_HANDLE)
  {
    hdDisableDevice(ghHD);
    ghHD = HD_INVALID_HANDLE;
  }
}

/**
 * @Function initHD()
 *  Initializes the HDAPI.  This involves initing a device configuration, enabling
 *  forces, and scheduling a haptic thread callback for servicing the device.
 */
void initHD()
{
  HDErrorInfo error;
  ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    fprintf(stderr, "\nPress any key to quit.\n");
    getchar();
    exit(-1);
  }

  hdEnable(HD_FORCE_OUTPUT);

  hUpdateDeviceCallback = hdScheduleAsynchronous(
      hapticCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to start the scheduler");
    exit(-1);
  }
}

/**
 * @Callback_Function hapticCallback
 *  This is the main haptic rendering callback.  This callback will render an
 *  anchored spring force when the user presses the button.
 */
HDCallbackCode HDCALLBACK hapticCallback(void *pUserData)
{
  static const HDdouble kAnchorStiffness = 0.1;
  hduVector3Dd start_position, position;
  hduVector3Dd force = { 0, 0, 0 };
  HDdouble transform[16];
  int currentButtons, lastButtons;
  HDdouble forceClamp;
  HDErrorInfo error;

  hdBeginFrame(ghHD);

  hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);
  hdGetIntegerv(HD_LAST_BUTTONS, &lastButtons);
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

  // Detect button state transitions.
  if ((currentButtons & HD_DEVICE_BUTTON_1) != 0 &&
      (lastButtons & HD_DEVICE_BUTTON_1) == 0)
  {
    state.isAnchorActive = HD_TRUE;
    hdGetDoublev(HD_CURRENT_POSITION, state.start_position);
  }
  else if ((currentButtons & HD_DEVICE_BUTTON_1) == 0 &&
           (lastButtons & HD_DEVICE_BUTTON_1) != 0)
  {
    state.isAnchorActive = HD_FALSE;
  }

  // Force Computing and Clamping
  if (state.isAnchorActive)
  {
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_TRANSFORM, state.HD_transform);

    // Compute force that will attact the device towards the anchor position:
    // F = k * (anchor - position)
    hduVecSubtract(force, state.anchor, position);
    hduVecScaleInPlace(force, kAnchorStiffness);

    // Clamp the force.
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &forceClamp);
    if (hduVecMagnitude(force) > forceClamp)
    {
      hduVecNormalizeInPlace(force);
      hduVecScaleInPlace(force, forceClamp);
    }
  }
  else
  {
    force.set(0,0,0);
    position.set(0,0,0);
    state.start_position.set(0,0,0);
  }

  // Update the haptic device state
  hduVecSubtract(state.Delta_position, position, state.start_position);

  // Render the force
  hdSetDoublev(HD_CURRENT_FORCE, force);

  //time_now += 0.001;

  hdEndFrame(ghHD);

  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    if (hduIsForceError(&error))
    {
      /* Disable the anchor following the force error. */
      //
    }
    else
    {
      /* This is likely a more serious error, so bail. */
      hduPrintError(stderr, &error, "Error during haptic rendering");
      exit(-1);
    }
  }

  return HD_CALLBACK_CONTINUE;
}

/* --------- End of the File ----------- */

