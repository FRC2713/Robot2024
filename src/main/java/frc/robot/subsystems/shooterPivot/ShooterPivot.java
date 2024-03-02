package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.VehicleState;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends SubsystemBase {
  private static final LoggedTunableNumber intakingAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Intake Angle Degrees", 45);
  private static final LoggedTunableNumber fenderShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Fender Shot Angle Degrees", 48);
  private static final LoggedTunableNumber podiumShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Podium Shot Angle Degrees", 27.13);
  private static final LoggedTunableNumber ampShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Amp Shot Angle Degrees", -30);

  private static final LoggedTunableNumber atGoalThresholdDegrees =
      new LoggedTunableNumber("ShooterPivot/At Goal Threshold Degrees", 2);

  @RequiredArgsConstructor
  public enum State {
    INTAKING(intakingAngleDegrees), // also for outtaking
    FENDER_SHOT(fenderShotAngleDegrees),
    PODIUM_SHOT(podiumShotAngleDegrees),
    DYNAMIC_AIM(() -> VehicleState.getInstance().getDynamicPivotAngle().getDegrees()),
    AMP_SHOT(ampShotAngleDegrees),
    AUTO_SHOT_NonAmpSide_1(fenderShotAngleDegrees),
    AUTO_SHOT_NonAmpSide_2(fenderShotAngleDegrees),
    OFF(() -> 0);

    private final DoubleSupplier pivotAngleDegrees;
  }

  @Getter
  @AutoLogOutput(key = "ShooterPivot/State")
  private State state = State.OFF;

  public final ShooterPivotInputsAutoLogged inputs;
  private final ShooterPivotIO IO;

  public ShooterPivot(ShooterPivotIO IO) {
    this.inputs = new ShooterPivotInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("ShooterPivot", inputs);
    Logger.recordOutput("ShooterPivot/Target", state.pivotAngleDegrees.getAsDouble());

    if (state != State.OFF) {
      IO.setTargetAngle(state.pivotAngleDegrees.getAsDouble());
    }
  }

  public double getCurrentAngle() {
    return inputs.absoluteEncoderAdjustedAngle;
  }

  @AutoLogOutput(key = "ShooterPivot/isAtTargetAngle")
  public boolean isAtTargetAngle() {
    return (Math.abs(inputs.angleDegreesLeft - this.state.pivotAngleDegrees.getAsDouble())
            < atGoalThresholdDegrees.get())
        && (Math.abs(inputs.angleDegreesRight - this.state.pivotAngleDegrees.getAsDouble())
            < atGoalThresholdDegrees.get());
  }

  public static class Commands {
    public static Command setMotionMode(State mode) {
      return new InstantCommand(() -> Robot.shooterPivot.state = mode);
    }

    public static Command setModeAndWait(State mode) {
      return Commands.setMotionMode(mode)
          .repeatedly()
          .until(() -> (Robot.shooterPivot.isAtTargetAngle()));
    }
  }
}
