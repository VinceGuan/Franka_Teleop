//
// Created by yuan on 09/07/2021.
//

#include <iostream>
#include <HD/hd.h>
#include <HL/hl.h>

#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

int calibrationStyle;

void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT){}
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      hdUpdateCalibration(calibrationStyle);
    }
  }
}

int main() {

  std::cout << "Hello" << std::endl;

  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    return -1;
  }

  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    return -1;
  }
  HHD_Auto_Calibration();

  hdStopScheduler();
  hdDisableDevice(hHD);

  std::cout << "Done" << std::endl;

  return 0;

}