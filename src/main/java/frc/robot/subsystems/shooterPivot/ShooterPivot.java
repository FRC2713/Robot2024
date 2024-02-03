package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
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
  public enum FourBarMode {
    CLOSED_LOOP,
    OPEN_LOOP,
  }

  private FourBarMode mode;

  private final ProfiledPIDController ShooterPivotController;
  public final ShooterPivotInputsAutoLogged inputs;
  private final ShooterPivotIO IO;
  private final ArmFeedforward ff;
  private double targetDegs = 20;
  private LoggableMotor motor = new LoggableMotor("ShooterPivot", DCMotor.getNEO(1));

  public ShooterPivot(ShooterPivotIO IO) {

    this.ff = Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createArmFeedForward();
    this.ShooterPivotController =
        Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createProfiledPIDController(
            new Constraints(100, 200));
    this.inputs = new ShooterPivotInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
    mode = FourBarMode.CLOSED_LOOP;
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
        /*boolean shouldReset =
            Math.abs(inputs.absoluteEncoderAdjustedAngle - inputs.angleDegreesOne) > 3;
        if (shouldReset) {
          // reseed();
        }*/

        double effort =
            ShooterPivotController.calculate(
                Units.degreesToRadians(inputs.absoluteEncoderAdjustedAngle),
                Units.degreesToRadians(targetDegs));

        var goal = ShooterPivotController.getGoal();
        var setpoint = ShooterPivotController.getSetpoint();

        Logger.recordOutput("ShooterPivot/Goal/Position", Units.radiansToDegrees(goal.position));
        Logger.recordOutput("ShooterPivot/Goal/Velocity", Units.radiansToDegrees(goal.velocity));
        Logger.recordOutput(
            "ShooterPivot/Setpoint/Position", Units.radiansToDegrees(setpoint.position));
        Logger.recordOutput(
            "ShooterPivot/Setpoint/Velocity", Units.radiansToDegrees(setpoint.velocity));

        // Logger.recordOutput("ShooterPivot/Should Reseed", shouldReset);

        Logger.recordOutput("ShooterPivot/Control Effort", effort);

        double ffEffort = ff.calculate(setpoint.position, setpoint.velocity);
        Logger.recordOutput("ShooterPivot/FF Effort", ffEffort);

        effort += ffEffort;
        effort = MathUtil.clamp(effort, -12, 12);
        voltage = effort;
        Logger.recordOutput("ShooterPivot/Total Effort", effort);

        double current = ShooterPivotConstants.SHOOTER_PIVOT_MAX_CURRENT;
        // MathUtil.clamp(
        //     Math.abs(currentController.calculate(inputs.angleDegreesOne, targetDegs))
        //         + ShooterPivotConstants.FOUR_BAR_BASE_CURRENT,
        //     0,
        //     ShooterPivotConstants.FOUR_BAR_MAX_CURRENT);
        IO.setCurrentLimit((int) current);
        // Logger.recordOutput("ShooterPivot/Current Limit", current);
        break;
      case OPEN_LOOP:
        // System.out.println(Robot.operator.getLeftX());
        // double deltaDegrees = Robot.operator.getLeftX();
        // this.targetDegs += deltaDegrees;
        voltage = -1 * Robot.operator.getLeftX() * ShooterPivotConstants.MAX_DEGREES_PER_SECOND;
        break;
    }

    IO.setVoltage(voltage);
    IO.updateInputs(inputs);
  }

  public double getCurrentAngle() {
    return this.inputs.absoluteEncoderAdjustedAngle;
  }

  public void setGoal(double goal) {
    this.targetDegs = goal;
    this.IO.setPosition(goal);
  }

  public static class Commands {
    public static Command setTargetAngle(SuperStructureBuilder structure) {
      return new InstantCommand(
          () -> {
            Robot.shooterPivot.setTargetAngle(structure.getShooterPivotAngleDegrees());
          });
      // if()
      // Robot.elevator.s

    }
  }
}
