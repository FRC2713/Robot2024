package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;
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
  private double targetDegs = Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES;
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

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("4Bar", inputs);
    motor.log(inputs.currentDrawOne, inputs.outputVoltage);

    double voltage = 0;
    switch (mode) {
      case CLOSED_LOOP:
        {
          boolean shouldReset =
              Math.abs(inputs.absoluteEncoderAdjustedAngle - inputs.angleDegreesOne) > 3;
          if (shouldReset) {
            // reseed();
          }
          double effort =
              ShooterPivotController.calculate(
                  // absoluteEncoderAdjustedAngle, angleDegreesOne
                  Units.degreesToRadians(inputs.absoluteEncoderAdjustedAngle),
                  Units.degreesToRadians(targetDegs));

          var goal = ShooterPivotController.getGoal();
          var setpoint = ShooterPivotController.getSetpoint();

          Logger.recordOutput("4Bar/Goal/Position", Units.radiansToDegrees(goal.position));
          Logger.recordOutput("4Bar/Goal/Velocity", Units.radiansToDegrees(goal.velocity));
          Logger.recordOutput("4Bar/Setpoint/Position", Units.radiansToDegrees(setpoint.position));
          Logger.recordOutput("4Bar/Setpoint/Velocity", Units.radiansToDegrees(setpoint.velocity));

          Logger.recordOutput("4Bar/Should Reseed", shouldReset);

          Logger.recordOutput("4Bar/Control Effort", effort);

          double ffEffort = ff.calculate(setpoint.position, setpoint.velocity);
          Logger.recordOutput("4Bar/FF Effort", ffEffort);

          effort += ffEffort;
          effort = MathUtil.clamp(effort, -12, 12);
          Logger.recordOutput("4Bar/Total Effort", effort);

          voltage = effort;

          double current = ShooterPivotConstants.SHOOTER_PIVOT_MAX_CURRENT;
          // MathUtil.clamp(
          //     Math.abs(currentController.calculate(inputs.angleDegreesOne, targetDegs))
          //         + ShooterPivotConstants.FOUR_BAR_BASE_CURRENT,
          //     0,
          //     ShooterPivotConstants.FOUR_BAR_MAX_CURRENT);
          IO.setCurrentLimit((int) current);
          Logger.recordOutput("4Bar/Current Limit", current);
        }
        break;
      case OPEN_LOOP:
        voltage = -1 * Robot.operator.getLeftX();
        break;
    }
  }
}
