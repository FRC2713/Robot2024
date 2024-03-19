package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
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
      new LoggedTunableNumber("Shooter/Differential RPM", 1000);

  private static final LoggedTunableNumber holdingGpShooterRpm =
      new LoggedTunableNumber("Shooter/Resting RPM", 0);
  private static final LoggedTunableNumber holdingFeederVolts =
      new LoggedTunableNumber("Shooter/Resting Feeder Volts", 0);

  private static final LoggedTunableNumber intakingShooterRpm =
      new LoggedTunableNumber("Shooter/Intaking Feeder RPM", 0);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Shooter/Intaking Feeder Volts", 6);

  private static final LoggedTunableNumber outtakingShooterRpm =
      new LoggedTunableNumber("Shooter/Outtaking Shooter RPM", 4000);
  private static final LoggedTunableNumber outtakingFeederVolts =
      new LoggedTunableNumber("Shooter/Outtaking Feeder Volts", 12);

  private static final LoggedTunableNumber fullInShooterRPM =
      new LoggedTunableNumber("Shooter/Full-In Shooter RPM", 4000);
  private static final LoggedTunableNumber fullInFeederVolts =
      new LoggedTunableNumber("Shooter/Full-In Feeder Volts", 12);

  private static final LoggedTunableNumber fullOutShooterRPM =
      new LoggedTunableNumber("Shooter/Full_Out Shooter RPM", -4000);
  private static final LoggedTunableNumber fullOutFeederVolts =
      new LoggedTunableNumber("Shooter/Full-Out Feeder Volts", -12);

  private static final LoggedTunableNumber ampShotShooterRPM =
      new LoggedTunableNumber("Shooter/Amp Shot Shooter RPM", 3000);
  private static final LoggedTunableNumber ampShotFeederVolts =
      new LoggedTunableNumber("Shooter/Amp Shot Feeder Volts", 5);

  private static final LoggedTunableNumber elevatorShotShooterRPM =
      new LoggedTunableNumber("Shooter/Elevator Shooter RPM", 4000);
  private static final LoggedTunableNumber elevatorShotFeederVolts =
      new LoggedTunableNumber("Shooter/Elevator Feeder Volts", 12);

  private static final LoggedTunableNumber preSpinRPM =
      new LoggedTunableNumber("Shooter/Pre-spin RPM", fenderShotShooterRpm.get() * 0.75);

  private static final LoggedTunableNumber atGoalThresholdRPM =
      new LoggedTunableNumber("Shooter/At Goal Threshold RPM", 200);

  private static final LoggedTunableNumber feederShotRPM =
      new LoggedTunableNumber("Shooter/Feeder Shot RPM", 4000);

  private static final double WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE = 0.1;
  private final Debouncer debouncer =
      new Debouncer(WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE, DebounceType.kRising);

  public enum State {
    FENDER_SHOT(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        () -> 0,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    PODIUM_SHOT(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        shooterDifferentialRpm,
        podiumShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    ELEVATOR_SHOT(
        elevatorShotShooterRPM,
        elevatorShotShooterRPM,
        shooterDifferentialRpm,
        elevatorShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    HOLDING_GP(
        holdingGpShooterRpm,
        holdingGpShooterRpm,
        () -> 0,
        holdingFeederVolts,
        () -> true,
        () -> true),
    INTAKING(
        intakingShooterRpm,
        intakingShooterRpm,
        () -> 0,
        intakingFeederVolts,
        () -> true,
        () -> true),
    OUTTAKE_FORWARD(
        outtakingShooterRpm, shooterDifferentialRpm, () -> 0, outtakingFeederVolts, () -> true),
    FULL_OUT(fullOutShooterRPM, fullOutShooterRPM, () -> 0, fullOutFeederVolts, () -> true),
    FULL_IN(fullInShooterRPM, fullInShooterRPM, () -> 0, fullInFeederVolts, () -> true),
    AMP_SHOT(ampShotShooterRPM, ampShotShooterRPM, () -> 0, ampShotFeederVolts, () -> true),
    AUTO_SHOT_NonAmpSide_1(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        shooterDifferentialRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    AUTO_SHOT_NonAmpSide_2(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        shooterDifferentialRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    FORCE_MANUAL_CONTROL(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        shooterDifferentialRpm,
        () -> Robot.operator.getLeftY() * 12, // [-1, 1] * 12V
        () -> true),
    PRE_SPIN(preSpinRPM, preSpinRPM, () -> 0, () -> 0, () -> true),
    OFF(() -> 0, () -> 0, () -> 0, () -> 0, () -> true),
    OUTTAKE_BACKWARDS(() -> -1000, () -> -1000, () -> 0, () -> -3, () -> true),
    CLEANING(() -> 10, () -> 10, () -> 0, () -> 1, () -> true),
    FEEDER_SHOT(
        feederShotRPM,
        feederShotRPM,
        shooterDifferentialRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle());
    private final DoubleSupplier leftRpm, rightRpm, differentialRpm, feederVolts;
    private final BooleanSupplier additionalFeederCondition, limitSwitchOn;

    private State(
        DoubleSupplier leftRpm,
        DoubleSupplier rightRpm,
        DoubleSupplier differentialRpm,
        DoubleSupplier feederVolts,
        BooleanSupplier additionalFeederCondition) {
      this.leftRpm = leftRpm;
      this.rightRpm = rightRpm;
      this.differentialRpm = differentialRpm;
      this.feederVolts = feederVolts;
      this.additionalFeederCondition = additionalFeederCondition;
      this.limitSwitchOn = () -> false;
    }

    private State(
        DoubleSupplier leftRpm,
        DoubleSupplier rightRpm,
        DoubleSupplier differentialRpm,
        DoubleSupplier feederVolts,
        BooleanSupplier additionalFeederCondition,
        BooleanSupplier limitSwitchOn) {
      this.leftRpm = leftRpm;
      this.rightRpm = rightRpm;
      this.differentialRpm = differentialRpm;
      this.feederVolts = feederVolts;
      this.additionalFeederCondition = additionalFeederCondition;
      this.limitSwitchOn = limitSwitchOn;
    }
  }

  @Setter
  @Getter
  @AutoLogOutput(key = "Shooter/State")
  public State state = State.OFF;

  private final ShooterIO IO;
  public final ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs, state);
    IO.setDisableOnLimitSwitch(true);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs, state);

    boolean shouldSpinFeeder = debouncer.calculate(isAtTarget());
    Logger.recordOutput("Shooter/Should spin feeder", shouldSpinFeeder);

    if (state == State.INTAKING && hasGamePiece()) {
      state = State.HOLDING_GP;
    }

    IO.setDisableOnLimitSwitch(state.limitSwitchOn.getAsBoolean());

    if ((shouldSpinFeeder && state.additionalFeederCondition.getAsBoolean())
        || this.state == Shooter.State.FORCE_MANUAL_CONTROL) {
      IO.setFeederVolts(state.feederVolts.getAsDouble());
    } else {
      IO.setFeederVolts(0.0);
    }

    if (state == State.OFF) {
      IO.setShooterVolts(0, 0);
    } else {
      this.setShooterRpms(
          state.leftRpm.getAsDouble(),
          state.rightRpm.getAsDouble(),
          state.differentialRpm.getAsDouble());
    }
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * @param leftRPM
   * @param rightRPM
   * @param differentialRpm
   */
  private void setShooterRpms(double leftRPM, double rightRPM, double differentialRpm) {
    IO.setMotorSetPoint(
        state.leftRpm.getAsDouble() + differentialRpm,
        state.rightRpm.getAsDouble() - differentialRpm);
  }

  @AutoLogOutput(key = "Shooter/isAtTarget")
  public boolean isAtTarget() {
    // double leftTarget =
    // state.leftRpm.getAsDouble()
    // + shooterDifferentialRpm.getAsDouble()
    // - atGoalThresholdRPM.getAsDouble();
    // double rightTarget =
    // state.rightRpm.getAsDouble()
    // - shooterDifferentialRpm.getAsDouble()
    // - atGoalThresholdRPM.getAsDouble();

    // return inputs.leftSpeedRPM > leftTarget && inputs.rightSpeedRPM >
    // rightTarget;

    double differential = state.differentialRpm.getAsDouble();

    // if (state == State.FEEDING) {
    // differential = 0;
    // }

    double leftError = (inputs.leftSpeedRPM) - (state.leftRpm.getAsDouble() + differential);
    double rightError = (inputs.rightSpeedRPM) - (state.rightRpm.getAsDouble() - differential);

    Logger.recordOutput("Shooter/Left error", leftError);
    Logger.recordOutput("Shooter/Right error", rightError);

    return Math.abs(leftError) < atGoalThresholdRPM.get()
        && Math.abs(rightError) < atGoalThresholdRPM.get();
  }

  @AutoLogOutput(key = "Shooter/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.LSTripped;
  }

  public static class Commands {}
}
