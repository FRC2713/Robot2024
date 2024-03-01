package com.revrobotics;

public class CANSparkMaxHandle {
  public final long handle;

  public CANSparkMaxHandle(CANSparkMax sparkMax) {
    handle = sparkMax.sparkMaxHandle;
  }
}
