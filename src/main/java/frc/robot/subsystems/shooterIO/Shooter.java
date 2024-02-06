package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.rhr.RHRPIDFFController;
import frc.robot.util.LoggableMotor;
import frc.robot.util.SuperStructureBuilder;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;
  private LoggableMotor leftMotor;
  private LoggableMotor rightMotor;
  private static final RHRPIDFFController leftFlyWheelController =
      Constants.ShooterConstants.MOTOR_GAINS;
  private static final RHRPIDFFController rightFlyWheelController =
      Constants.ShooterConstants.MOTOR_GAINS;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs);
    SmartDashboard.putData(leftFlyWheelController);
    SmartDashboard.putData(rightFlyWheelController);
    leftMotor = new LoggableMotor("LeftMotor", DCMotor.getNeoVortex(1));
    rightMotor = new LoggableMotor("rightMotor", DCMotor.getNeoVortex(1));
  }

  @Override
  public void periodic() {
    Logger.processInputs("Shooter", inputs);
    IO.updateInputs(inputs);
    // IO.setRightMotorRPMSetPoint(rightFlyWheelTargetRPM);
    // IO.setLeftMotorRPMSetPoint(leftFlyWheelTargetRPM);

    double effortRight =
        rightFlyWheelController.calculate(
            inputs.rightFlyWheelSpeedRPM, rightFlyWheelController.getSetpoint());
    double effortLeft =
        rightFlyWheelController.calculate(
            inputs.leftFlyWheelSpeedRPM, leftFlyWheelController.getSetpoint());

    effortLeft = MathUtil.clamp(effortLeft, -12, 12);
    effortRight = MathUtil.clamp(effortRight, -12, 12);

    IO.setLeftVoltage(effortLeft);
    IO.setRightVoltage(effortRight);
  }

  public void setLeftFlyWheelTargetRPM(double leftFlyTargetRPM) {
    leftFlyWheelController.setSetpoint(leftFlyTargetRPM);
  }

  public void setRightFLyWheelTargetRPM(double rightTargetRPM) {
    rightFlyWheelController.setSetpoint(rightTargetRPM);
  }

  public static class Commands {
    public static Command setTargetRPM(double targetRPM) {
      return new InstantCommand(
          () -> {
            Robot.shooter.setRightFLyWheelTargetRPM(targetRPM);
            Robot.shooter.setLeftFlyWheelTargetRPM(targetRPM);
          });
    }

    public static Command toRPM(SuperStructureBuilder builder) {
      return new InstantCommand(
          () -> {
            Robot.shooter.setLeftFlyWheelTargetRPM(builder.getShooterMotorSpeed());
            Robot.shooter.setRightFLyWheelTargetRPM(builder.getShooterMotorSpeed());
          });
    }
  }
}
