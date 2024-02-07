package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SuperStructureBuilder;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP,
  }

  private ControlMode mode;

  public final ShooterPivotInputsAutoLogged inputs;
  private final ShooterPivotIO IO;
  private double targetDegs = 0;
  private LoggableMotor motor = new LoggableMotor("ShooterPivot", DCMotor.getNEO(1));

  public ShooterPivot(ShooterPivotIO IO) {
    this.inputs = new ShooterPivotInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
    mode = ControlMode.CLOSED_LOOP;
  }

  public void setTargetAngle(double angleInDegrees) {
    if (angleInDegrees > Constants.ShooterPivotConstants.MAX_ANGLE_DEGREES) {
      RedHawkUtil.ErrHandler.getInstance().addError("targetToHeight");
      this.targetDegs =
          MathUtil.clamp(
              angleInDegrees,
              Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES,
              Constants.ShooterPivotConstants.MAX_ANGLE_DEGREES);
      return;
    }
    this.targetDegs = angleInDegrees;
  }

  public void periodic() {
    motor.log(inputs.currentDrawOne, inputs.outputVoltage);
    Logger.processInputs("ShooterPivot", inputs);
    double voltage = 0;

    Logger.recordOutput("ShooterPivot/Mode", mode);
    switch (mode) {
      case CLOSED_LOOP:
        IO.setTargetPosition(this.targetDegs);
        break;
      case OPEN_LOOP:
        voltage = -1 * Robot.operator.getLeftX() * ShooterPivotConstants.MAX_DEGREES_PER_SECOND;
        IO.setVoltage(voltage);
        break;
    }

    IO.updateInputs(inputs);
    Logger.recordOutput("ShooterPivot/isAtTarget", this.isAtTargetAngle());
  }

  public boolean isAtTargetAngle() {
    return (Math.abs(getCurrentAngle() - this.targetDegs) < 0.001);
  }

  public double getCurrentAngle() {
    return this.inputs.absoluteEncoderAdjustedAngle;
  }

  public void setGoal(double goal) {
    this.targetDegs = goal;
  }

  public static class Commands {
    public static Command setTargetAngle(SuperStructureBuilder structure) {
      return new InstantCommand(
          () -> {
            Robot.shooterPivot.setTargetAngle(structure.getShooterPivotAngleDegrees());
          });
    }

    public static Command setTargetAngle(double angleDegrees) {
      return new InstantCommand(
          () -> {
            Robot.shooterPivot.setTargetAngle(angleDegrees);
          });
    }

    public static Command setTargetAndWait(double angleDegrees) {
      return setTargetAngle(angleDegrees).until(Robot.shooterPivot::isAtTargetAngle);
    }
  }
}
