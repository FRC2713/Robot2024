package frc.robot.subsystems.swerveIO.module;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public class SwerveModuleInputs {
    public double aziAbsoluteEncoderRawVolts = 0.0;
    public double aziAbsoluteEncoderAdjVolts = 0.0;
    public double aziAbsoluteEncoderAdjAngleDeg = 0.0;

    public double aziOutputVolts = 0.0;
    public double aziCurrentDrawAmps = 0.0;
    public double aziEncoderPositionDeg = 0.0;
    public double aziEncoderSimplifiedPositionDeg = 0.0;
    public double aziEncoderVelocityDegPerSecond = 0.0;
    public double aziTempCelcius;
    public double driveEncoderPositionMetres = 0.0;
    public double driveEncoderVelocityMetresPerSecond = 0.0;
    public double driveOutputVolts = 0.0;
    public double driveCurrentDrawAmps = 0.0;
    public double driveTempCelcius = 0.0;
  }

  /**
   * Updates SwerveModule input values with given SwerveModuleInputs instance
   *
   * @param inputs SwerveModuleInputs instance
   */
  public void updateInputs(SwerveModuleInputs inputs);

  /**
   * Sets the azimuth of the current SwerveModule to the given volts
   *
   * @param aziVolts volts to set the azimuth to
   */
  public void setAzimuthVoltage(double aziVolts);

  /**
   * Sets the drive of the current SwerveModule to the given volts
   *
   * @param driveVolts volts to set the drive to
   */
  public void setDriveVoltage(double driveVolts);

  public void seed();
}
