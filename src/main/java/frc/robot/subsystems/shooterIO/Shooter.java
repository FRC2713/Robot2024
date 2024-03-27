package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber noDifferentialShotRPM =
      new LoggedTunableNumber("Shooter/Fender Shot RPM", 4000);
  private static final LoggedTunableNumber differentialShotRPM =
      new LoggedTunableNumber("Shooter/Fender Shot RPM", 4000);

  private static final LoggedTunableNumber genericFeederVolts =
      new LoggedTunableNumber("Shooter/Fender Shot Feeder Volts", 12);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Shooter/Intaking Feeder Volts", 8.5);
  private static final LoggedTunableNumber offFeederVolts =
      new LoggedTunableNumber("Shooter/Resting Feeder Volts", 0);

  /**
   * Applies a differential speed to the left and right wheels. Positive values make the left wheel
   * go faster and the right wheel slower Negative values make the left wheel slower and the right
   * wheel faster.
   */
  private static final LoggedTunableNumber shooterDifferentialRPM =
      new LoggedTunableNumber("Shooter/Differential RPM", 1000);

  private static final LoggedTunableNumber preSpinRPM =
      new LoggedTunableNumber("Shooter/Pre-spin RPM", noDifferentialShotRPM.get() * 0.75);

  private static final LoggedTunableNumber atGoalThresholdRPM =
      new LoggedTunableNumber("Shooter/At Goal Threshold RPM", 210);

  private static final double WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE = 0.1;
  private final Debouncer debouncer =
      new Debouncer(WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE, DebounceType.kRising);

  public enum ShooterState {
    NO_DIFFERENTIAL_SHOT(noDifferentialShotRPM, noDifferentialShotRPM, () -> 0),
    DIFFERENTIAL_SHOT(differentialShotRPM, differentialShotRPM, shooterDifferentialRPM),
    PRE_SPIN(preSpinRPM, preSpinRPM, () -> 0),
    OFF(() -> 0, () -> 0, () -> 0);
    private final DoubleSupplier leftRpm, rightRpm, differentialRpm;

    private ShooterState(
        DoubleSupplier leftRpm, DoubleSupplier rightRpm, DoubleSupplier differentialRpm) {
      this.leftRpm = leftRpm;
      this.rightRpm = rightRpm;
      this.differentialRpm = differentialRpm;
    }
  }

  public enum FeederState {
    FEED_SHOT(genericFeederVolts, () -> false),
    INTAKE(intakingFeederVolts, () -> true),
    HOLDING_GP(offFeederVolts, () -> false),
    OFF(offFeederVolts, () -> false);
    private final DoubleSupplier feederVolts;
    private final BooleanSupplier limitSwitchEnabled;

    private FeederState(DoubleSupplier feederVolts, BooleanSupplier limitSwitchEnabled) {
      this.feederVolts = feederVolts;
      this.limitSwitchEnabled = limitSwitchEnabled;
    }
  }

  @Setter
  @Getter
  @AutoLogOutput(key = "Shooter/ShooterState")
  ShooterState shooterState = ShooterState.OFF;

  @Setter
  @Getter
  @AutoLogOutput(key = "Shooter/FeederState")
  FeederState feederState = FeederState.OFF;

  private final ShooterIO IO;
  public final ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs, shooterState, feederState);
    IO.setDisableOnLimitSwitch(true);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs, shooterState, feederState);

    boolean shouldSpinFeeder = debouncer.calculate(isAtTarget());
    Logger.recordOutput("Shooter/Should spin feeder", shouldSpinFeeder);

    if (feederState == FeederState.INTAKE && hasGamePiece()) {
      feederState = FeederState.HOLDING_GP;
    }

    IO.setDisableOnLimitSwitch(feederState.limitSwitchEnabled.getAsBoolean());

    // if ((shouldSpinFeeder && state.additionalFeederCondition.getAsBoolean())
    //     || this.state == Shooter.State.FORCE_MANUAL_CONTROL) {
    //   IO.setFeederVolts(state.feederVolts.getAsDouble());
    // } else {
    //   IO.setFeederVolts(0.0);
    // }

    IO.setFeederVolts(0.0);

    if (shooterState == ShooterState.OFF) {
      IO.setShooterVolts(0, 0);
    } else {
      this.setShooterRpms(
          shooterState.leftRpm.getAsDouble(),
          shooterState.rightRpm.getAsDouble(),
          shooterState.differentialRpm.getAsDouble());
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
        shooterState.leftRpm.getAsDouble() + differentialRpm,
        shooterState.rightRpm.getAsDouble() - differentialRpm);
  }

  @AutoLogOutput(key = "Shooter/isAtTarget")
  public boolean isAtTarget() {

    double differential = shooterState.differentialRpm.getAsDouble();

    double leftError = (inputs.leftSpeedRPM) - (shooterState.leftRpm.getAsDouble() + differential);
    double rightError =
        (inputs.rightSpeedRPM) - (shooterState.rightRpm.getAsDouble() - differential);

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
