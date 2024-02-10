package frc.robot.subsystems.feederIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.LoggableMotor;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  @Getter private LoggableMotor feederMotor;

  public enum MotionMode {
    INTAKE_GP,
    HOLD_GAMEPIECE,
    SEND_TO_SHOOTER,
    OFF;
  }

  @Setter public MotionMode motionMode = MotionMode.OFF;
  private FeederIO IO;
  private double target;
  private FeederInputsAutoLogged inputs;

  public Feeder(FeederIO IO) {
    this.IO = IO;
    this.inputs = new FeederInputsAutoLogged();
    this.feederMotor = new LoggableMotor("Feeder", DCMotor.getNEO(1));
  }

  public void setTarget(double targetRPM) {
    target = targetRPM;
    IO.setSetpoint(targetRPM);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);
    Logger.recordOutput("Feeder/TargetRPM", target);
    Logger.recordOutput("Feeder/atTarget", atTarget());
    Logger.processInputs("Feeder", inputs);

    switch (motionMode) {
      case INTAKE_GP:
        Robot.intake.setMotionMode(Intake.MotionMode.SEND_GP_TO_FEEDER);
        setTarget(2000);
        if (hasGamepiece()) {
          motionMode = MotionMode.HOLD_GAMEPIECE;
        }
        break;
      case HOLD_GAMEPIECE:
        setTarget(0);
        break;
      case SEND_TO_SHOOTER:
        setTarget(2000);
        if (!hasGamepiece()) {
          motionMode = MotionMode.OFF;
        }
        break;
      case OFF:
        if (hasGamepiece()) {
          motionMode = MotionMode.HOLD_GAMEPIECE;
        }
      default:
        setTarget(0);
        break;
    }
  }

  public boolean hasGamepiece() {
    return IO.hasGamepiece();
  }

  public boolean atTarget() {
    return Math.abs(inputs.velocityRPM - target) < 0.1;
  }

  public static class Commands {
    public static Command setToVelocity(double rpm) {
      return new InstantCommand(() -> Robot.feeder.setTarget(rpm));
    }

    public static Command setToVelocityAndWait(double rpm) {
      return new SequentialCommandGroup(
          setToVelocity(rpm), new WaitUntilCommand(Robot.feeder::atTarget));
    }

    public static Command setMotionMode(MotionMode mode) {
      return new InstantCommand(() -> Robot.feeder.setMotionMode(mode));
    }
  }
}
