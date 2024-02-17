package frc.robot.subsystems.intakeIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import lombok.Setter;

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

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    // IO.updateInputs(inputs);
    this.IO = IO;
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

    // double lDesiredVoltage =
    //     leftRpm
    //         / (Constants.IntakeConstants.MAX_RPM)
    //         * RobotController.getBatteryVoltage(); // is this what we want to do?
    // double rDesiredVoltage =
    //     rightRPM / (Constants.IntakeConstants.MAX_RPM) * RobotController.getBatteryVoltage();

    // Logger.recordOutput("Intake/Left Applied Volts", lDesiredVoltage);
    // Logger.recordOutput("Intake/Right Applied Volts", rDesiredVoltage);
    IO.setVoltage(0, 0);
  }

  public boolean hasGamepiece() {
    return false;
    // return IO.hasGamepiece();
  }

  public void periodic() {
    // IO.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);

    // boolean hasGamepiece = hasGamepiece();
    // boolean leftIsAtTarget = leftIsAtTarget();
    // boolean rightIsAtTarget = rightIsAtTarget();

    // Logger.recordOutput("Intake/Sensor Range", this.inputs.sensorRange);

    // Logger.recordOutput("Intake/Left Target RPM", leftTargetRPM);
    // Logger.recordOutput("Intake/Left Has reached target", leftIsAtTarget);

    // Logger.recordOutput("Intake/Right Target RPM", rightTargetRPM);
    // Logger.recordOutput("Intake/Right Has reached target", rightIsAtTarget);

    // Logger.recordOutput("Intake/Has gamepiece", hasGamepiece);
    // SmartDashboard.putBoolean("Has gamepiece?", hasGamepiece);
    // Logger.recordOutput("Intake/Mode", motionMode);

    // switch (motionMode) {
    //   case HOLDING_GP:
    //     setRPM(0);
    //     break;
    //   case INTAKE_GP:
    //     // setRPM(4000);
    //     IO.setVoltage(7, 7);
    //     if (hasGamepiece()) {
    //       motionMode = MotionMode.HOLDING_GP;
    //       // RumbleManager.getInstance().setDriver(1, 2);
    //     }
    //     break;
    //   case SEND_GP_TO_FEEDER:
    //     // setRPM(2000);
    //     IO.setVoltage(5, 5);
    //     // if (!hasGamepiece()) {
    //     //   motionMode = MotionMode.OFF;
    //     // }
    //     break;
    //   case OUTAKE_GP:
    //     IO.setVoltage(-6, -6);
    //     // if (!hasGamepiece()) {
    //     // motionMode = MotionMode.OFF;
    //     // }
    //     break;
    //   case OFF:
    //     if (hasGamepiece()) {
    //       motionMode = MotionMode.HOLDING_GP;
    //     }
    //   default:
    //     IO.setVoltage(0, 0);
    //     break;
    // }
  }

  public static class Commands {
    public static Command setMotionMode(MotionMode mode) {
      return new InstantCommand(() -> Robot.intake.setMotionMode(mode));
    }

    public static Command setVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setRPM(targetRPM, targetRPM));
    }

    // public static Command setVelocityRPMAndWait(double targetRPM) {
    //   return new SequentialCommandGroup(
    //       setVelocityRPM(targetRPM),
    //       new WaitUntilCommand(
    //           () -> Robot.intake.leftIsAtTarget() && Robot.intake.rightIsAtTarget()));
    // }

    // public static Command setVelocityRPMUntilGP(double targetRPM) {
    //   return setVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.hasGamepiece());
    // }
  }
}
