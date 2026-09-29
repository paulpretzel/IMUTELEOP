// calibration.h -- per-board gyro calibration.
//
// Run tools/gyro_scale_cal.py with this board and paste its output over
// GYRO_CAL_M. The identity matrix below means "uncalibrated". The MPU6050's
// scale error (up to +-3 %) and cross-axis error (up to +-2 %) turn directly
// into heading drift during large arm motions, so this is worth doing.
#pragma once

// omega_calibrated = GYRO_CAL_M * (omega_raw - bias), row-major.
static const float GYRO_CAL_M[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

// Optional prior for the gyro bias temperature slope [deg/s per deg C]. The
// firmware refines this online from rest periods, so zeros are fine; copy the
// "slope" values from the serial [STAT] log after a long warm-up session to
// give the next boot a head start.
static const float GYRO_TEMP_SLOPE[3] = {0.0f, 0.0f, 0.0f};
