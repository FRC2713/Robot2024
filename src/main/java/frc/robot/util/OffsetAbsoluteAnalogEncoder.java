package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class OffsetAbsoluteAnalogEncoder {

  private final double voltageOffset;
  private AnalogEncoder analogEncoder;

  private LinearFilter offsetStabilizer = LinearFilter.movingAverage(20);

  public OffsetAbsoluteAnalogEncoder(int port, double voltageOffset) {
    this.analogEncoder = new AnalogEncoder(port);
    this.voltageOffset = voltageOffset;
  }

  public OffsetAbsoluteAnalogEncoder(int port) {
    this(port, 0.0);
  }

  public double getUnadjustedVoltage() {
    return analogEncoder.getAbsolutePosition();
  }

  public double getAdjustedVoltage() {
    return analogEncoder.getAbsolutePosition() - voltageOffset;
  }

  public double getFilteredOffset() {
    return offsetStabilizer.calculate(analogEncoder.getAbsolutePosition());
  }

  public Rotation2d getUnadjustedRotation2d() {
    return Rotation2d.fromDegrees(getAdjustedVoltage() * 360.0);
  }

  public Rotation2d getAdjustedRotation2d() {
    return simplifyRotation2d(getUnadjustedRotation2d());
  }

  /**
   * Given a Rotation2d object, returns a new Rotation2d object adjusted to a value between -180 and
   * 180 degrees.
   *
   * @param rotation A rotation2d object to be simplified.
   * @return A rotation2d object with a degree measure between -180 and 180.
   */
  public static Rotation2d simplifyRotation2d(Rotation2d rotation) {
    double unadjustedDegrees = rotation.getDegrees();
    double min = -180, max = 180;

    while (unadjustedDegrees < min) {
      unadjustedDegrees += 360;
    }

    while (unadjustedDegrees >= max) {
      unadjustedDegrees -= 360;
    }

    return Rotation2d.fromDegrees(unadjustedDegrees);
  }
}
