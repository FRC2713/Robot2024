package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.rhr.RHRPIDFFController;
import org.littletonrobotics.junction.Logger;

public class SwerveHeadingController {
  private static SwerveHeadingController instance;
  private Rotation2d setpoint;
  private RHRPIDFFController controller;
  private double error;
  private TunableNT4 tunableSetpoint;

  private SwerveHeadingController() {
    controller = DriveConstants.K_HEADING_CONTROLLER_GAINS.createRHRController();
    controller.enableContinuousInput(0, 360);

    setpoint = Robot.swerveDrive.getUsablePose().getRotation();
    tunableSetpoint = new TunableNT4("Heading Controller/Setpoint", setpoint.getDegrees());
    tunableSetpoint.addHook(
        x -> {
          setSetpoint(Rotation2d.fromDegrees(x));
        });
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
    this.setpoint =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? setpoint
            : Rotation2d.fromDegrees((setpoint.getDegrees() + 180) % 360);
  }

  public void addToSetpoint(Rotation2d setpoint) {
    this.setpoint = this.setpoint.plus(setpoint);
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    return this.error < 0.1;
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

    if (!controller.atSetpoint()) {
      Rotation2d currentHeading = Robot.swerveDrive.getYaw();
      output = controller.calculate(currentHeading.getDegrees(), setpoint.getDegrees());
      output =
          MathUtil.clamp(
              output,
              -Units.radiansToDegrees(DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC),
              Units.radiansToDegrees(DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC));
      error = setpoint.getDegrees() - currentHeading.getDegrees();
      Logger.recordOutput("Heading Controller/error", error);

      // if ((Math.abs(error) <= 1) || (Math.abs(error) >= 359 && Math.abs(error) <= 360)) {
      // return 0;
      // }
    }

    Logger.recordOutput("Heading Controller/update", output);
    return output;
  }
}
