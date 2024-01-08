package frc.robot.subsystems.swerveIO.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;
import frc.robot.util.PIDFFController;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  SwerveModuleIO io;
  public final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

  private final PIDFFController driveController;
  private final PIDFFController azimuthController;

  SwerveModuleState state;
  public final ModuleInfo information;

  LoggableMotor azimuthMotor, driveMotor;

  /**
   * Creates a new SwerveModule object.
   *
   * @param swerveModuleIO The IO layer. Change this to change which motor controllers you're using
   *     (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param name The name of the swerve module (how it shows up in logging tools)
   */
  public SwerveModule(SwerveModuleIO swerveModuleIO, ModuleInfo information) {
    this.information = information;
    this.driveController = new PIDFFController(this.information.getDriveGains());
    this.azimuthController = new PIDFFController(this.information.getAzimuthGains());

    io = swerveModuleIO;
    io.updateInputs(inputs);

    state = new SwerveModuleState(0, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
    azimuthController.enableContinuousInput(-180, 180);

    azimuthMotor =
        new LoggableMotor("Swerve " + information.getName() + " Azimuth", DCMotor.getNEO(1));
    driveMotor = new LoggableMotor("Swerve " + information.getName() + " Drive", DCMotor.getNEO(1));
  }

  /**
   * Returns the current objective state of the swerve drive.
   *
   * @return The desired SwerveModuleState object.
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(
        inputs.driveEncoderVelocityMetresPerSecond,
        Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  public SwerveModuleState getDesiredState() {
    return state;
  }

  public void seed() {
    io.seed();
  }

  private void recordOutput(String key, double value) {
    Logger.getInstance().recordOutput("Swerve/" + information.getName() + '/' + key, value);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.driveEncoderPositionMetres, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  // Only used to characterize the drive
  public double getVoltageAppliedForCharacterization() {
    return inputs.driveOutputVolts;
  }

  public double getAziCurrentDraw() {
    return inputs.aziCurrentDrawAmps;
  }

  public double getDriveCurrentDraw() {
    return inputs.driveCurrentDrawAmps;
  }

  public double getTotalCurrentDraw() {
    return inputs.driveCurrentDrawAmps + inputs.aziCurrentDrawAmps;
  }

  // Only used to characterize the drive
  public void applyVoltageForCharacterization(double voltage) {
    io.setDriveVoltage(voltage);
  }

  /**
   * Optimizes the given SwerveModuleState and make it the setpoint of the swerve module.
   *
   * @param desiredState The new setpoint of the swerve module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    state =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  /**
   * Recalculates the voltage outputs of the drive and azimuth voltages and sets them. Should run on
   * every code loop, so put it in periodic() for best results.
   */
  public void update() {
    final double driveOutput =
        driveController.calculate(
            inputs.driveEncoderVelocityMetresPerSecond, state.speedMetersPerSecond);

    boolean useMotorEncoder = Math.abs(inputs.aziEncoderPositionDeg) > 0.1 || Robot.isSimulation();
    boolean useAbsoluteEncoder = Math.abs(inputs.aziAbsoluteEncoderRawVolts) > 0.1;
    double feedbackVal;
    if (useMotorEncoder) {
      feedbackVal = inputs.aziEncoderPositionDeg;
      Logger.getInstance().recordOutput("Azimuth feedback source", "motor");
    } else if (useAbsoluteEncoder) {
      feedbackVal = inputs.aziAbsoluteEncoderAdjAngleDeg;
      Logger.getInstance().recordOutput("Azimuth feedback source", "absolute encoder");
    } else {
      feedbackVal = inputs.aziEncoderPositionDeg;
      Logger.getInstance().recordOutput("Azimuth feedback source", "motor backup");
    }

    final double turnOutput = azimuthController.calculate(feedbackVal, state.angle.getDegrees());

    recordOutput("Desired Drive Volts", driveOutput);
    recordOutput("Desired Azi Volts", turnOutput);

    io.setDriveVoltage(driveOutput);
    io.setAzimuthVoltage(turnOutput);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    azimuthMotor.log(inputs.aziCurrentDrawAmps, inputs.aziOutputVolts);
    driveMotor.log(inputs.driveCurrentDrawAmps, inputs.driveOutputVolts);
    update();

    Logger.getInstance().processInputs("Swerve/" + information.getName(), inputs);
    recordOutput("Azimuth Error", state.angle.getDegrees() - inputs.aziEncoderPositionDeg);
    recordOutput(
        "Drive Error", state.speedMetersPerSecond - inputs.driveEncoderVelocityMetresPerSecond);
    recordOutput("Target Speed", state.speedMetersPerSecond);
    recordOutput("Angle Speed", state.angle.getDegrees());
    recordOutput(
        "Azimuth Encoder Delta",
        inputs.aziEncoderPositionDeg - inputs.aziAbsoluteEncoderAdjAngleDeg);
  }
}
