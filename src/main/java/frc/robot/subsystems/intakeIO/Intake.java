package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
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

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Has gamepiece", hasGamepiece);
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {

    public static Command setVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setRPM(targetRPM, targetRPM));
    }

    public static Command setVelocityRPMAndWait(double targetRPM) {
      return setVelocityRPM(targetRPM)
          .repeatedly()
          .until(() -> Robot.intake.leftIsAtTarget() && Robot.intake.rightIsAtTarget());
    }

    public static Command setVelocityRPMUntilGP(double targetRPM) {
      return setVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.hasGamepiece());
    }
  }
}
