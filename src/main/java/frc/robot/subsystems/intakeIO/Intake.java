package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.subsystems.feederIO.Feeder.FeederState;
import frc.robot.util.LoggableMotor;
import frc.robot.util.RumbleManager;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum MotionMode {
    INTAKE_GP,
    HOLDING_GP,
    SEND_GP_TO_FEEDER,
    OUTAKE_GP,
    OFF
  }

  @Setter public MotionMode motionMode = MotionMode.OFF;

  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private double leftTargetRPM, rightTargetRPM = 0.0;
  private LoggableMotor leftMotor, rightMotor;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;

    leftMotor = new LoggableMotor("Left intake Roller", DCMotor.getNEO(1));
    rightMotor = new LoggableMotor("Right intake Roller", DCMotor.getNEO(1));
  }

  public boolean leftIsAtTarget() {
    return Math.abs(inputs.leftVelocityRPM - leftTargetRPM) < 0.5;
  }

  public boolean rightIsAtTarget() {
    return Math.abs(inputs.rightVelocityRPM - rightTargetRPM) < 0.5;
  }

  public void setRPM(double rpm) {
    setRPM(rpm, rpm);
  }

  public void setRPM(double leftRpm, double rightRPM) {
    this.leftTargetRPM = leftRpm;
    this.rightTargetRPM = rightRPM;

    double lDesiredVoltage =
        leftRpm
            / (Constants.IntakeConstants.MAX_RPM)
            * RobotController.getBatteryVoltage(); // is this what we want to do?
    double rDesiredVoltage =
        rightRPM / (Constants.IntakeConstants.MAX_RPM) * RobotController.getBatteryVoltage();

    Logger.recordOutput("Intake/Left Applied Volts", lDesiredVoltage);
    Logger.recordOutput("Intake/Right Applied Volts", rDesiredVoltage);
    IO.setVoltage(lDesiredVoltage, rDesiredVoltage);
  }

  public boolean hasGamepiece() {
    return IO.hasGamepiece();
  }

  public void periodic() {
    IO.updateInputs(inputs);
    leftMotor.log(inputs.leftCurrentAmps, inputs.leftOutputVoltage);
    rightMotor.log(inputs.rightCurrentAmps, inputs.rightOutputVoltage);

    boolean hasGamepiece = hasGamepiece();
    boolean leftIsAtTarget = leftIsAtTarget();
    boolean rightIsAtTarget = rightIsAtTarget();

    Logger.recordOutput("Intake/Sensor Range", this.inputs.sensorRange);

    Logger.recordOutput("Intake/Left Target RPM", leftTargetRPM);
    Logger.recordOutput("Intake/Left Has reached target", leftIsAtTarget);

    Logger.recordOutput("Intake/Right Target RPM", rightTargetRPM);
    Logger.recordOutput("Intake/Right Has reached target", rightIsAtTarget);

    Logger.recordOutput("Intake/Has gamepiece", hasGamepiece);

    Logger.recordOutput("Intake/Mode", motionMode);

    switch (motionMode) {
      case HOLDING_GP:
        setRPM(0);
        break;
      case INTAKE_GP:
        setRPM(4000);
        if (hasGamepiece()) {
          motionMode = MotionMode.HOLDING_GP;
          RumbleManager.getInstance().setDriver(1, 2);
        }
        break;
      case SEND_GP_TO_FEEDER:
        setRPM(2000);
        if (!hasGamepiece()) {
          motionMode = MotionMode.OFF;
        }
        break;
      case OUTAKE_GP:
        setRPM(-4000);
        if (!hasGamepiece()) {
          motionMode = MotionMode.OFF;
        }
        break;
      case OFF:
      default:
        setRPM(0, 0);
        break;
    }

    Logger.processInputs("Intake", inputs);
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {
    public static Command setMotionMode(MotionMode mode) {
      return new InstantCommand(() -> Robot.intake.setMotionMode(mode));
    }

    public static Command setVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setRPM(targetRPM, targetRPM));
    }

    public static Command setVelocityRPMAndWait(double targetRPM) {
      return new SequentialCommandGroup(
          setVelocityRPM(targetRPM),
          new WaitUntilCommand(
              () -> Robot.intake.leftIsAtTarget() && Robot.intake.rightIsAtTarget()));
    }

    public static Command setVelocityRPMUntilGP(double targetRPM) {
      return setVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.hasGamepiece());
    }
  }
}
