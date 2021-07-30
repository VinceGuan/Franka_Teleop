// Copyright (c) 2017 Franka Emika GmbH; Bristol Robotics Laboratory (UK).
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*****************************************************************************
Author: Yuan (Vincent) Guan
Email: yuan2.guan@brl.ac.uk / vincentguan@ieee.org
 For questions, comments or bug reports, please send email or check the Github
 repo at:
    https://github.com/VinceGuan/Franka_Teleop/tree/main/examples
File:
  HD_Franks_Simple_Motion_Mirror.cpp
Description:
  an example showing how to teleoperate the Franka Panda Robot using 3DSystem
  Touch haptic device. This application is currently a LTS version.
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
#include <SDL2/SDL.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>

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
  hduVector3Dd Feedback_Force;
} HapticState;

/* <Function> */
void exitHandler();
void initHD();
void Check_KeyEvent();
//
HDCallbackCode HDCALLBACK hapticCallback(void *pUserData);

/* <Global> */
//Eigen::MatrixXd pinv_Eigen_SVD(Eigen::Map<const Eigen::Matrix<double, 6, 7>> map);
Eigen::MatrixXd pinv_Eigen_SVD(Eigen::MatrixXd &origin);
// double time_now = 0;
SDL_Event Event;
HapticState state;
SDL_Window *window = nullptr;


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
  /// -----------------------------------------------------------------------------------
  /////////////////////////////////// <START> Init window ///////////////////////////////
  if(SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "SDL Video Initialisation Error: " << SDL_GetError() << std::endl;
  }
  else {
    window = SDL_CreateWindow("A Window Title",SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,640,480,SDL_WINDOW_SHOWN);
    if (window == nullptr) {
      std::cout << "SDL Window Initialisation Error: " << SDL_GetError() << std::endl;
    }
    else {
      SDL_UpdateWindowSurface(window);
    }
  }
  //////////////////////////////////// <END> Init window ////////////////////////////////

  /// -----------------------------------------------------------------------------------
  ////////////////// <START> Register the Exit signal handle ////////////////////////////
  if (std::atexit(exitHandler) != 0) {
    std::cout << "Failed to register the Exit Handler" << std::endl;
  }
  /////////////////// <END> Register the Exit signal handle ///////////////////////

  /// -----------------------------------------------------------------------------------
  ///////////////////////// <START> Creating new thread ///////////////////////
  std::thread Check_KeyEvent_thread(Check_KeyEvent);
  ////////////////////////// <END> Creating new thread ////////////////////////

  /// -----------------------------------------------------------------------------------
  ///////////////////////// <START> Init HD API and find device ///////////////////////
  initHD();
  std::cout << "HDAPI Initialised." << std::endl;
  ///////////////////////// <START> Init HD API and find device ///////////////////////

  /// -----------------------------------------------------------------------------------

  // TODO: raise a flag of state data ready. after that we can do the control.
  std::cout << state.Delta_position << std::endl;

  /// -----------------------------------------------------------------------------------
  ////////////////////////// <START> Initiate Stiffness and Damping matrix ////////////////////////
  // Compliance parameters
  const double translational_stiffness{1000.0};
  const double rotational_stiffness{30.0};
  const double null_stiffness{20.0};
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
  /// -----------------------------------------------------------------------------------

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    // get the robot initial state
    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    // TODO: Usage of Eigen::Map (reuse memory of a variable as Eigen type)
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    /// -----------------------------------------------------------------------------------
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

    /// -----------------------------------------------------------------------------------
    // set collision behavior
    robot.setCollisionBehavior({{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                               {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                               {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                               {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}});

    double time = 0.0;
    std::array<double, 16> initial_pose;
    while (1) {
      try {
        /// -----------------------------------------------------------------------------------
        ////////////////// <START> Define torque controller callback ////////////////////////
        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [&](const franka::RobotState& robot_state,
                                             franka::Duration duration) -> franka::Torques {
          // Get starting timepoint
          auto start = std::chrono::high_resolution_clock::now();

          // get state variables
          std::array<double, 7> coriolis_array = model.coriolis(robot_state);
          std::array<double, 42> jacobian_array =
              model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

          // convert to Eigen
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 1>> tau_ext(robot_state.O_F_ext_hat_K.data());
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Affine3d transform_hd;
          Eigen::Affine3d transform_offset;
          Eigen::Vector3d position(transform.translation());
          Eigen::Quaterniond orientation(transform.linear());

          if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
          }

          time += duration.toSec();

          position_d(0) = initial_transform.translation()[0] + 0.002 * state.Delta_position[0];
          position_d(1) = initial_transform.translation()[1] - 0.002 * state.Delta_position[2];
          position_d(2) = initial_transform.translation()[2] + 0.002 * state.Delta_position[1];

          // TODO: Orientation
          if (state.isAnchorActive) {
             transform_hd = Eigen::Matrix4d::Map(state.HD_transform);
//             std::cout << transform.linear() << "  " << std::endl;
             Eigen::AngleAxisd rot_90_x (M_PI/2, Eigen::Vector3d(1, 0, 0));
             orientation_d = rot_90_x.toRotationMatrix() * transform_hd.linear() * rot_90_x.toRotationMatrix(); 
//             orientation_d = (initial_transform * transform.linear()).linear();
//             Eigen::Transform<double,3,2> t = Eigen::AngleAxis(&rotation_vector);
             std::cout << rot_90_x.toRotationMatrix() * transform_hd.linear() * rot_90_x.toRotationMatrix() << std::endl;
          
           // std::cout <<  transform_hd.linear()  << std::endl;
          }

          /// compute error to desired equilibrium pose
          /// position error
          Eigen::Matrix<double, 6, 1> error;
          error.head(3) << position - position_d;

          /// orientation error
          // "difference" quaternion
          // std::cout << orientation_d.coeffs() << std::endl;
          if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
          }
          // "difference" quaternion
          Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
          error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
          // Transform to base frame
          error.tail(3) << -transform.linear() * error.tail(3);

          // TODO: null space
          double null_error;
          null_error = q(0) - 0.0;
          double task_null = null_stiffness * null_error + 2.0 * dq(0);

          Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
          Eigen::MatrixXd pinv_jacobian =
              jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
//                Eigen::MatrixXd pinv_jacobian = pinv_Eigen_SVD(jacob);
//           jacobian.completeOrthogonalDecomposition().pseudoInverse();

          // compute control
          Eigen::VectorXd tau_task(7), tau_d(7);

          // Spring damper system with damping ratio=1
          tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
          // tau_task.setZero();

          Eigen::MatrixXd q_null = Eigen::MatrixXd::Zero(7, 1);
          q_null(0) = -task_null;

          tau_d << tau_task + coriolis +
                       (I - jacobian.transpose() * pinv_jacobian.transpose()) * 2.0 * q_null;

          //////////////////////////////////////////
          // try this on first and then try SVD to compute.
          // jacobian.completeOrthogonalDecomposition().pseudoInverse();
          // pinv_Eigen_SVD(jacobian);
          ////////////////////////////////////////////

          // TODO: Assign the feedback force
          /*Eigen::Matrix<double, 6 ,1> Wrench_ext;
    //      Wrench_ext = (stiffness * error + damping * (jacobian * dq));
          Wrench_ext = jacobian * tau_d;
          const double force_feedback_ratio(-0.5);
          state.Feedback_Force.set(force_feedback_ratio * Wrench_ext[0], force_feedback_ratio *
    Wrench_ext[2], force_feedback_ratio * Wrench_ext[1]);
    //      std::cout << state.Feedback_Force.magnitude() << std::endl;*/

          Eigen::Matrix<double, 6, 1> Wrench_ext(tau_ext);
          const double force_feedback_ratio(-0.1);
          state.Feedback_Force.set(force_feedback_ratio * Wrench_ext[0],
                                   force_feedback_ratio * Wrench_ext[2],
                                   force_feedback_ratio * Wrench_ext[1]);

          std::array<double, 7> tau_d_array{};
          Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

          // Toggle this to enter the guiding mode
          // tau_d_array = {{0,0,0,0,0,0,0}};

          if (!isRunning) {
            std::cout << "Here finishsssss" << std::endl;
            return franka::MotionFinished(franka::Torques(tau_d_array));
          }

          auto stop = std::chrono::high_resolution_clock::now();

          auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

//          std::cout << dur.count() << std::endl;

          return tau_d_array;
        };

        ///////////////////// <END> Define torque controller callback /////////////////////////

        /// -----------------------------------------------------------------------------------
        ////////////////// <START> Ask the user to start the control loop /////////////////////
        // start real-time control loop
        //    std::cout << "WARNING: Collision thresholds are set to high values. "
        //              << "Make sure you have the user stop at hand!" << std::endl
        //              << "After starting try to push the robot and see how it reacts." << std::endl
        //              << "Press Enter to continue..." << std::endl;
        //    std::cin.ignore();

        // pass the controller callback function ptr to the control
        robot.control(impedance_control_callback);
        /////////////////// <END> Ask the user to start the control loop /////////////////////

      } catch (const franka::Exception& ex) {
        // print exception
        std::cout << ex.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot.automaticErrorRecovery();
      }
    }

    /// -----------------------------------------------------------------------------------
//    while (isRunning) {
//      // TODO:wait....
//      std::cout << "is running3" << std::endl;
//    }

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  std::cout << "Please wait, we will quit the program..." << std::endl;
  //return 0;
  std::exit(0);
}


/**
 * @Function Check_KeyEvent
 *  Function used to monitor the keyboard events.
 * @in Null
 * @return Null
 */
void Check_KeyEvent ()
{
  std::cout << "is running1" << std::endl;
  while (isRunning) {
    while (SDL_PollEvent(&Event) != 0) {
      if (Event.type == SDL_KEYDOWN) {
        switch (Event.key.keysym.sym) {
          case SDLK_q:
            std::cout << "is running2" << SDL_PollEvent(&Event) << std::endl;
            isRunning = false;
            std::exit(0);
            //break;
        }
      }
    }
  }
}

/**
 * @Function exitHandler()
 *  The handler get called when the application is exiting to shutdown the HD API.
 */
void exitHandler()
{
  std::cout << "On Exit ..." << std::endl;
  hdStopScheduler();
  hdUnschedule(hUpdateDeviceCallback);
  if (ghHD != HD_INVALID_HANDLE)
  {
    hdDisableDevice(ghHD);
    ghHD = HD_INVALID_HANDLE;
  }
  std::cout << "Safely quit the haptic device ..." << std::endl;
  SDL_DestroyWindow(window);
  SDL_Quit();
  std::cout << "Safely quit the SDL and destroy the window ..." << std::endl;
  std::cout << "Now quit ..." << std::endl;
}

/**
 * @Function initHD()
 *  Initializes the HDAPI.  This involves initing a device configuration, enabling
 *  forces, and scheduling a haptic thread callback for servicing the device.
 */
void initHD()
{
  // To capture any HD Error message
  HDErrorInfo error;
  // Init the HD_DEFAULT_DEVICE
  ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    fprintf(stderr, "\nPress any key to quit.\n");
    getchar();
    exit(-1);
  }

  // Enable the force feedback
  hdEnable(HD_FORCE_OUTPUT);

  // Creating Asynchronous callback handle
  // TODO: See if can use nullptr to replace nUserData.
  hUpdateDeviceCallback = hdScheduleAsynchronous(
      hapticCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

  // Start the HD scheduler.
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to start the scheduler");
    exit(-1);
  }
}

/**
 * @Function pinv_eigen_based
 *  This is the main haptic rendering callback.  This callback will render an
 *  anchored spring force when the user presses the button.
 */
Eigen::MatrixXd pinv_Eigen_SVD(Eigen::MatrixXd &origin) {

  const float er = 0;
  // perform svd decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Build SVD decomposition results
  Eigen::MatrixXd U = svd_holder.matrixU();
  Eigen::MatrixXd V = svd_holder.matrixV();
  Eigen::MatrixXd D = svd_holder.singularValues();

  // Build the S matrix
  Eigen::MatrixXd S(V.cols(), U.cols());
  S.setZero();

  for (unsigned int i = 0; i < D.size(); ++i) {

    if (D(i, 0) > er) {
      S(i, i) = 1 / D(i, 0);
    } else {
      S(i, i) = 0;
    }
  }

  // pinv_matrix = V * S * U^T
  return V * S * U.transpose();
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

  /// -----------------------------------------------------------------------------------
  ///////////////////// <START> HD frame //////////////////////
  hdBeginFrame(ghHD);

  hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);
  hdGetIntegerv(HD_LAST_BUTTONS, &lastButtons);
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

//  std::cout << "------------" << std::endl;
//  std::cout << transform[0] << "  " << transform[4] << "  " << transform[8] << std::endl;
//  std::cout << transform[1] << "  " << transform[5] << "  " << transform[9] << std::endl;
//  std::cout << transform[2] << "  " << transform[6] << "  " << transform[10] << std::endl;



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


    force = state.Feedback_Force;
    hduVecScaleInPlace(force, 1.0);

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
  //std::cout << force << std::endl;

  //time_now += 0.001;

  hdEndFrame(ghHD);
  ///////////////////// <END> HD frame //////////////////////

  /// -----------------------------------------------------------------------------------
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

