package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Robot;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.util.SuperStructureBuilder;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends SubsystemBase {
  public enum MotionMode {
    CLOSED_LOOP,
    FEED_CLOSED_LOOP,
    OPEN_LOOP,
    SHORT_AUTO_SHOTS
  }

  public MotionMode mode = MotionMode.SHORT_AUTO_SHOTS;

  public final ShooterPivotInputsAutoLogged inputs;
  private final ShooterPivotIO IO;
  private double targetDegs = 0;

  public ShooterPivot(ShooterPivotIO IO) {
    this.inputs = new ShooterPivotInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
    mode = MotionMode.SHORT_AUTO_SHOTS;
  }

  public void setTargetAngle(double angleInDegrees) {
    this.targetDegs =
        MathUtil.clamp(
            angleInDegrees,
            Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES,
            Constants.ShooterPivotConstants.MAX_ANGLE_DEGREES);

    IO.setTargetPosition(angleInDegrees);
  }

  @Override
  public void periodic() {
    Logger.processInputs("ShooterPivot", inputs);
    double voltage = 0;

    Logger.recordOutput("ShooterPivot/Mode", mode);
    switch (mode) {
      case SHORT_AUTO_SHOTS:
        setTargetAngle((ShooterPivotConstants.SHORT_AUTO_SHOTS));
        break;
      case CLOSED_LOOP:
        setTargetAngle((this.targetDegs));
        break;
      case FEED_CLOSED_LOOP:
        setTargetAngle((ShooterPivotConstants.FEEDING_ANGLE));
        if (isAtTargetAngle()) {
          Robot.feeder.setMotionMode(Feeder.MotionMode.INTAKE_GP);
        }
        break;
      case OPEN_LOOP:
        voltage = -1 * Robot.operator.getLeftX() * ShooterPivotConstants.MAX_DEGREES_PER_SECOND;
        IO.setVoltage(voltage);
        break;
    }

    IO.updateInputs(inputs);
    Logger.recordOutput("ShooterPivot/isAtTarget", this.isAtTargetAngle());
    Logger.recordOutput("ShooterPivot/TargetAngleDegs", targetDegs);
  }

  public boolean isAtTargetAngle() {
    return (Math.abs(getCurrentAngle() - this.targetDegs) < 3);
  }

  public double getCurrentAngle() {
    return this.inputs.absoluteEncoderAdjustedAngle;
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
      return new SequentialCommandGroup(
          setTargetAngle(angleDegrees), new WaitUntilCommand(Robot.shooterPivot::isAtTargetAngle));
    }

    public static Command setMotionMode(MotionMode mode) {
      return new InstantCommand(() -> Robot.shooterPivot.mode = mode);
    }
  }
}
