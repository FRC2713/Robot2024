package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

public class CANSparkMaxExtensions {
  /**
   * Checked function for setting controller inverted.
   *
   * <p>This call has no effect if the controller is a follower. To invert a follower, see the
   * follow() method.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public static REVLibError setInverted(CANSparkMax sparkMax, boolean isInverted) {
    sparkMax.throwIfClosed();
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetInverted(sparkMax.sparkMaxHandle, isInverted));
  }

  /** Enable center aligned mode for the duty cycle sensor. */
  public static REVLibError enableCenterAlignedMode(CANSparkMax sparkMax) {
    CANSparkMaxHandle handle = new CANSparkMaxHandle(sparkMax);
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(handle.handle, 152, true));
  }

  /** Disable center aligned mode for the duty cycle sensor. */
  public static REVLibError disableCenterAlignedMode(CANSparkMax sparkMax) {
    CANSparkMaxHandle handle = new CANSparkMaxHandle(sparkMax);
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetParameterBool(handle.handle, 152, false));
  }

  /**
   * Enable mode which sets the output of the PID controllers to be voltage instead of duty cycle.
   *
   * <p>To disable, change or disable voltage compensation. Those settings will overwrite this one
   */
  public static REVLibError enablePIDVoltageOutput(CANSparkMax sparkMax) {
    CANSparkMaxHandle handle = new CANSparkMaxHandle(sparkMax);
    return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetParameterUint32(handle.handle, 74, 1));
  }
}
