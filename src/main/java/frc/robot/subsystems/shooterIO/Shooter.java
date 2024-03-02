package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber fenderShotShooterRpm =
      new LoggedTunableNumber("Shooter/Fender Shot RPM", 4000);
  private static final LoggedTunableNumber fenderShotFeederVolts =
      new LoggedTunableNumber("Shooter/Fender Shot Feeder Volts", 12);

  private static final LoggedTunableNumber podiumShotShooterRpm =
      new LoggedTunableNumber("Shooter/Fender Shot RPM", 4000);
  private static final LoggedTunableNumber podiumShotFeederVolts =
      new LoggedTunableNumber("Shooter/Fender Shot Feeder Volts", 12);

  /**
   * Applies a differential speed to the left and right wheels. Positive values make the left wheel
   * go faster and the right wheel slower Negative values make the left wheel slower and the right
   * wheel faster.
   */
  private static final LoggedTunableNumber shooterDifferentialRpm =
      new LoggedTunableNumber("Shooter/Differential RPM", 250);

  private static final LoggedTunableNumber holdingGpShooterRpm =
      new LoggedTunableNumber("Shooter/Resting RPM", 0);
  private static final LoggedTunableNumber holdingFeederVolts =
      new LoggedTunableNumber("Shooter/Resting Feeder Volts", 0);

  private static final LoggedTunableNumber intakingShooterRpm =
      new LoggedTunableNumber("Shooter/Intaking Feeder RPM", 0);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Shooter/Intaking Feeder Volts", 2);

  private static final LoggedTunableNumber outtakingShooterRpm =
      new LoggedTunableNumber("Shooter/Outtaking Shooter RPM", 0);
  private static final LoggedTunableNumber outtakingFeederVolts =
      new LoggedTunableNumber("Shooter/Outtaking Feeder Volts", -5);

  private static final LoggedTunableNumber ampShotShooterRMP =
      new LoggedTunableNumber("Shooter/Outtaking Shooter RPM", 0);
  private static final LoggedTunableNumber ampShotFeederVolts =
      new LoggedTunableNumber("Shooter/Outtaking Feeder Volts", -5);

  private static final LoggedTunableNumber preSpinRPM =
      new LoggedTunableNumber("Shooter/Pre-spin RPM", fenderShotShooterRpm.get() * 0.75);

  private static final LoggedTunableNumber atGoalThresholdRPM =
      new LoggedTunableNumber("Shooter/At Goal Threshold RPM", 200);

  private static final double WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE = 0.1;
  private final Debouncer debouncer =
      new Debouncer(WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE, DebounceType.kRising);

  @RequiredArgsConstructor
  public enum State {
    FENDER_SHOT(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle(),
        true),
    PODIUM_SHOT(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        podiumShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle(),
        true),
    AMP_SHOT(ampShotShooterRMP, ampShotShooterRMP, ampShotFeederVolts, () -> true, true),
    AUTO_SHOT_NonAmpSide_1(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        podiumShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle(),
        true),
    AUTO_SHOT_NonAmpSide_2(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        podiumShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle(),
        true),
    FORCE_MANUAL_CONTROL(
        () -> Robot.operator.getLeftX() * 4000, // [-1, 1] * 4000 rpm
        () -> Robot.operator.getLeftX() * 4000, // [-1, 1] * 4000 rpm
        () -> Robot.operator.getLeftY() * 12, // [-1, 1] * 12V
        () -> true,
        true),
    HOLDING_GP(holdingGpShooterRpm, holdingGpShooterRpm, holdingFeederVolts, () -> true, true),
    INTAKING(intakingShooterRpm, intakingShooterRpm, intakingFeederVolts, () -> true, true),
    OUTAKING(outtakingShooterRpm, outtakingShooterRpm, outtakingFeederVolts, () -> true, true),
    PRE_SPIN(preSpinRPM, preSpinRPM, () -> 0, () -> true, true),
    OFF(() -> 0, () -> 0, () -> 0, () -> true, true);
    private final DoubleSupplier leftRpm, rightRpm, feederRpm;
    private final BooleanSupplier additionalFeederCondition;
    private final boolean isShooting;
  }

  @Setter
  @Getter
  @AutoLogOutput(key = "Shooter/State")
  public State state = State.OFF;

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs, state);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs, state);

    boolean shouldSpin = debouncer.calculate(isAtTarget());
    Logger.recordOutput("Shooter/Should spin", shouldSpin);

    if (state == State.INTAKING && hasGamePiece()) {
      state = State.HOLDING_GP;
    }

    if (shouldSpin && state.additionalFeederCondition.getAsBoolean()) {
      IO.setFeederVolts(state.feederRpm.getAsDouble());
    } else {
      IO.setFeederVolts(0.0);
    }

    // if (state == State.FENDER_SHOT && !debouncer.calculate(hasGamePiece())) {
    // state = State.OFF;
    // }

    double differential = shooterDifferentialRpm.getAsDouble();

    if (state == State.OFF) {
      IO.setShooterVolts(0, 0);
    } else {
      IO.setMotorSetPoint(state.leftRpm.getAsDouble(), state.rightRpm.getAsDouble());
    }
    Logger.processInputs("Shooter", inputs);
  }

  @AutoLogOutput(key = "Shooter/isAtTarget")
  public boolean isAtTarget() {
    // double leftTarget =
    //     state.leftRpm.getAsDouble()
    //         + shooterDifferentialRpm.getAsDouble()
    //         - atGoalThresholdRPM.getAsDouble();
    // double rightTarget =
    //     state.rightRpm.getAsDouble()
    //         - shooterDifferentialRpm.getAsDouble()
    //         - atGoalThresholdRPM.getAsDouble();

    // return inputs.leftSpeedRPM > leftTarget && inputs.rightSpeedRPM > rightTarget;

    return Math.abs(state.leftRpm.getAsDouble() - inputs.leftSpeedRPM) < atGoalThresholdRPM.get()
        && Math.abs(state.rightRpm.getAsDouble() - inputs.rightSpeedRPM) < atGoalThresholdRPM.get();
  }

  @AutoLogOutput(key = "Shooter/hasGamePiece")
  public boolean hasGamePiece() {
    return (inputs.laserCanDistanceMM < 95);
  }

  @AutoLogOutput(key = "Shooter/gamePieceTooFar")
  public boolean gamePieceTooFar() {
    return (inputs.laserCan2farDistanceMM < 95 && state.ordinal() > 5);
  }

  public static class Commands {
    public static Command setState(State mode) {
      return new InstantCommand(() -> Robot.shooter.setState(mode));
    }
  }
}
