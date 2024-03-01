package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Robot;
import frc.robot.VehicleState;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends TrapezoidProfileSubsystem {
  private static final LoggedTunableNumber intakingAngleDegrees = new LoggedTunableNumber(
      "ShooterPivot/Intake Angle Degrees", 45);
  private static final LoggedTunableNumber fenderShotAngleDegrees = new LoggedTunableNumber(
      "ShooterPivot/Fender Shot Angle Degrees", 48);
  private static final LoggedTunableNumber podiumShotAngleDegrees = new LoggedTunableNumber(
      "ShooterPivot/Podium Shot Angle Degrees", 27.13);
  private static final LoggedTunableNumber ampShotAngleDegrees = new LoggedTunableNumber(
      "ShooterPivot/Amp Shot Angle Degrees", -30);
  private static final LoggedTunableNumber atGoalThresholdDegrees = new LoggedTunableNumber(
      "ShooterPivot/At Goal Threshold Degrees", 2);

  private final ArmFeedforward feedforward = new ArmFeedforward(
      ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKS(),
      ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKG(),
      ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKV());

  @RequiredArgsConstructor
  public enum State {
    INTAKING(intakingAngleDegrees), // also for outtaking
    FENDER_SHOT(fenderShotAngleDegrees),
    PODIUM_SHOT(podiumShotAngleDegrees),
    DYNAMIC_AIM(() -> VehicleState.getInstance().getDynamicPivotAngle().getDegrees()),
    AMP_SHOT(ampShotAngleDegrees),
    AUTO_SHOT_NonAmpSide_1(podiumShotAngleDegrees),
    AUTO_SHOT_NonAmpSide_2(podiumShotAngleDegrees),
    OFF(() -> 0);

    private final DoubleSupplier pivotAngleDegrees;
  }

  @Getter
  @AutoLogOutput(key = "ShooterPivot/State")
  private State state = State.OFF;

  public final ShooterPivotInputsAutoLogged inputs;
  private final ShooterPivotIO IO;

  public ShooterPivot(ShooterPivotIO IO) {
    super(
        new TrapezoidProfile.Constraints(
            ShooterPivotConstants.MAX_VELOCITY_RAD_SEC, ShooterPivotConstants.MAX_ACCEL_RAD_SEC_2),
        ShooterPivotConstants.STARTING_ANGLE_RADS);
    this.setGoal(ShooterPivotConstants.STARTING_ANGLE_RADS);
    this.inputs = new ShooterPivotInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    super.periodic();
    IO.updateInputs(inputs);
    Logger.processInputs("ShooterPivot", inputs);
    Logger.recordOutput("ShooterPivot/Target", state.pivotAngleDegrees.getAsDouble());
  }

  public double getCurrentAngle() {
    return inputs.absoluteEncoderAdjustedAngle;
  }

  @AutoLogOutput(key = "ShooterPivot/isAtTargetAngle")
  public boolean isAtTargetAngle() {
    return (Math.abs(inputs.angleDegreesLeft - this.state.pivotAngleDegrees.getAsDouble()) < atGoalThresholdDegrees
        .get())
        && (Math.abs(inputs.angleDegreesRight - this.state.pivotAngleDegrees.getAsDouble()) < atGoalThresholdDegrees
            .get());
  }

  public static class Commands {
    public static Command setState(State state) {
      return new InstantCommand(
          () -> {
            Robot.shooterPivot.setGoal(
                Units.degreesToRadians(state.pivotAngleDegrees.getAsDouble()));
            Robot.shooterPivot.state = state;
          });
    }

    public static Command setStateAndWait(State state) {
      return Commands.setState(state)
          .alongWith(new WaitUntilCommand(() -> Robot.shooterPivot.isAtTargetAngle()));
    }
  }

  @Override
  protected void useState(edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {

    Logger.recordOutput("ShooterPivot/ProfileSetpoint", setpoint.position);
    double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    IO.setTargetAngle(Units.radiansToDegrees(setpoint.position), ff);
  }
}
