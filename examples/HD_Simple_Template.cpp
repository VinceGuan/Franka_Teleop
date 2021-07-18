//
// Created by yuan on 09/07/2021.
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <assert.h>

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <chrono>

#if defined(WIN32)
# include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle hUpdateDeviceCallback = HD_INVALID_HANDLE;

typedef struct
{
  hduVector3Dd position;
  HDdouble transform[16];
  hduVector3Dd anchor;
  HDboolean isAnchorActive;
} HapticState;

void exitHandler();
void initHD();

HDCallbackCode HDCALLBACK hapticCallback(void *pUserData);

double time_now = 0;
HapticState state;

/*******************************************************************************
 Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main()
{
  std::atexit(exitHandler);
  initHD();
  std::cout << "Press Any key to quit." << std::endl;

  // TODO: raise a flag of state data ready. after that we can do the control.
  std::cout << state.position << std::endl;

  std::cin.get();

  std::cout << "Please wait, we will quit the program..." << std::endl;

  return 0;
}


/*******************************************************************************
 This handler will get called when the application is exiting. This is our
 opportunity to cleanly shutdown the HD API.
*******************************************************************************/
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


/*******************************************************************************
 Initializes the HDAPI.  This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void initHD()
{
  HDErrorInfo error;
  ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    fprintf(stderr, "\nPress any key to quit.\n");
    getchar();
    exit(-1);
  }


  hdEnable(HD_FORCE_OUTPUT);

  hUpdateDeviceCallback = hdScheduleAsynchronous(
      hapticCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    hduPrintError(stderr, &error, "Failed to start the scheduler");
    exit(-1);
  }
}


/*******************************************************************************
 This is the main haptic rendering callback.  This callback will render an
 anchored spring force when the user presses the button.
*******************************************************************************/
HDCallbackCode HDCALLBACK hapticCallback(void *pUserData)
{

  hduVector3Dd position;
  hduVector3Dd force = { 0, 0, 0 };

  HDErrorInfo error;

  hdBeginFrame(ghHD);

  hdGetDoublev(HD_CURRENT_POSITION, position);
  state.position = position;
  //std::cout << position << std::endl;
  hduVector3Dd position_d = {0,-45,0};
  force = 0.1 * (position_d - position);

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
