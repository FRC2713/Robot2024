package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;
import frc.robot.util.SuperStructureBuilder;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;
  private LoggableMotor leftMotor;
  private LoggableMotor rightMotor;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs);
    leftMotor = new LoggableMotor("LeftMotor", DCMotor.getNeoVortex(1));
    rightMotor = new LoggableMotor("rightMotor", DCMotor.getNeoVortex(1));
  }

  @Override
  public void periodic() {
    Logger.processInputs("Shooter", inputs);
    IO.updateInputs(inputs);
  }

  public void setFlyWheelTargetRPM(double targetRPM) {
    IO.setMotorSetPoint(targetRPM);
  }

  public void setVoltage(double volts) {
    IO.setLeftVoltage(volts);
    IO.setRightVoltage(volts);
  }

  public static class Commands {
    public static Command setTargetRPM(double targetRPM) {
      return new InstantCommand(
          () -> {
            Robot.shooter.setFlyWheelTargetRPM(targetRPM);
          });
    }

    public static Command setVoltage(double volts) {
      return new InstantCommand(
          () -> {
            Robot.shooter.setVoltage(volts);
          });
    }

    public static Command toRPM(SuperStructureBuilder builder) {
      return new InstantCommand(
          () -> {
            Robot.shooter.setFlyWheelTargetRPM(builder.getShooterMotorSpeed());
          });
    }
  }
}
