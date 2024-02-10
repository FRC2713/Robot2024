package frc.robot.subsystems.shooterIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.util.SuperStructureBuilder;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum MotionMode {
    FENDER_SHOT_OPEN_LOOP,
    FENDER_SHOT_CLOSED_LOOP,
    VELOCITY_CLOSED_LOOP,
    OFF
  }

  @Setter public MotionMode motionMode = MotionMode.OFF;

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;

  private double targetRPM;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);

    switch (motionMode) {
      case FENDER_SHOT_OPEN_LOOP:
        setVoltage(4);
        break;
      case FENDER_SHOT_CLOSED_LOOP:
        setFlyWheelTargetRPM(3500);
        if (isAtTarget()) {
          Robot.feeder.setMotionMode(Feeder.MotionMode.SEND_TO_SHOOTER);
        }
        break;
      case VELOCITY_CLOSED_LOOP:
        break;
      case OFF:
      default:
        setVoltage(0);
        break;
    }

    Logger.recordOutput("Shooter/Mode", motionMode);
    Logger.recordOutput("Shooter/isAtTarget", isAtTarget());
    Logger.processInputs("Shooter", inputs);
  }

  public void setFlyWheelTargetRPM(double targetRPM) {
    this.targetRPM = targetRPM;
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    IO.setMotorSetPoint(targetRPM);
  }

  public void setVoltage(double volts) {
    IO.setLeftVoltage(volts);
    IO.setRightVoltage(volts);
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.leftSpeedRPM - targetRPM) < 90;
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

    public static Command setMotionMode(MotionMode mode) {
      return new InstantCommand(() -> Robot.shooter.setMotionMode(mode));
    }
  }
}
