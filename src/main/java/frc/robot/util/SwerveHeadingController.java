package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.rhr.RHRPIDFFController;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SwerveHeadingController {
  private static SwerveHeadingController instance;
  private Rotation2d setpoint;
  private RHRPIDFFController controller;
  private double error;
  private LoggedTunableNumber tunableSetpoint;
  private Debouncer debouncer;
  private boolean isWithinTarget;
  private double acceptableError = 0.1;

  private SwerveHeadingController() {
    controller = DriveConstants.K_HEADING_CONTROLLER_GAINS.createRHRController();
    controller.enableContinuousInput(0, 360);

    setpoint = Robot.swerveDrive.getRegularPose().getRotation();
    tunableSetpoint = new LoggedTunableNumber("Heading Controller/Setpoint", setpoint.getDegrees());
    debouncer = new Debouncer(0.5);
  }

  /**
   * Ensures the SwerveHeadingController is not created more than once.
   *
   * @return The SwerveHeadingController object.
   */
  public static SwerveHeadingController getInstance() {
    if (instance == null) {
      instance = new SwerveHeadingController();
    }

    return instance;
  }

  /**
   * Changes the setpoint of the heading controller. (Note that this value is not loaded into the
   * PID controller until update() is called.)
   *
   * @param setpoint The new setpoint of the heading controller.
   */
  public void setSetpoint(Rotation2d setpoint) {
    // this.setpoint =
    //     DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
    //         ? setpoint
    //         : Rotation2d.fromDegrees((setpoint.getDegrees() + 180) % 360);

    this.setpoint = setpoint;
  }

  public void addToSetpoint(Rotation2d setpoint) {
    this.setpoint = this.setpoint.plus(setpoint);
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    if (this.acceptableError != 0.1) {
      this.acceptableError = 0.1;
      return false;
    }
    return isWithinTarget;
  }

  public boolean atSetpoint(double acceptableError) {
    // if (this.acceptableError != acceptableError) {
    //   this.acceptableError = acceptableError;
    //   return false;
    // }
    // return isWithinTarget;

    return this.error < acceptableError;
  }
  /**
   * Updates the heading controller PID with the setpoint and calculates output.
   *
   * @return The speed, in degrees per second, of rotation.
   */
  public double update() {
    Logger.recordOutput("Heading Controller/setpoint degrees", setpoint.getDegrees());
    SmartDashboard.putBoolean("Heading Controller/at setpoint", controller.atSetpoint());

    controller.setSetpoint(setpoint.getDegrees());
    Logger.recordOutput("Heading Controller/setpoint", setpoint.getDegrees());

    double output = 0;

    Rotation2d currentHeading = Robot.swerveDrive.getYaw();
    output = controller.calculate(currentHeading.getDegrees(), setpoint.getDegrees());
    output =
        MathUtil.clamp(
            output,
            -Units.radiansToDegrees(DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC),
            Units.radiansToDegrees(DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC));
    error = setpoint.getDegrees() - (currentHeading.getDegrees() % 360);
    Logger.recordOutput("Heading Controller/error", error);

    var chassisSpeeds = Robot.swerveDrive.getChassisSpeeds();
    if ((Math.abs(error) <= 1 || Math.abs(error - 360) <= 1)
        && chassisSpeeds.vxMetersPerSecond <= 0.25
        && chassisSpeeds.vyMetersPerSecond <= 0.25) {
      output = 0;
    }

    // if ((Math.abs(error) <= 1) || (Math.abs(error) >= 359 && Math.abs(error) <= 360)) {
    // return 0;
    // }

    Logger.recordOutput("Heading Controller/update", output);

    isWithinTarget = debouncer.calculate(this.error < this.acceptableError);

    return output;
  }
}
