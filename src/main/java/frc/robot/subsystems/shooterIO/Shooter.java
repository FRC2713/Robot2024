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
      new LoggedTunableNumber("Shooter/Differential RPM", 1000);

  private static final LoggedTunableNumber holdingGpShooterRpm =
      new LoggedTunableNumber("Shooter/Resting RPM", 0);
  private static final LoggedTunableNumber holdingFeederVolts =
      new LoggedTunableNumber("Shooter/Resting Feeder Volts", 0);

  private static final LoggedTunableNumber intakingShooterRpm =
      new LoggedTunableNumber("Shooter/Intaking Feeder RPM", 0);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Shooter/Intaking Feeder Volts", 3);

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
      new LoggedTunableNumber("Shooter/Amp Shot Shooter RPM", -1000);
  private static final LoggedTunableNumber ampShotFeederVolts =
      new LoggedTunableNumber("Shooter/Amp Shot Feeder Volts", -5);

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

  /*@RequiredArgsConstructor
  public enum State {
    FENDER_SHOT(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    PODIUM_SHOT(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        podiumShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    ELEVATOR_SHOT(
        elevatorShotShooterRPM,
        elevatorShotShooterRPM,
        elevatorShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    HOLDING_GP(holdingGpShooterRpm, holdingGpShooterRpm, holdingFeederVolts, () -> true),
    INTAKING(intakingShooterRpm, intakingShooterRpm, intakingFeederVolts, () -> true),
    OUTTAKE_FORWARD(outtakingShooterRpm, outtakingShooterRpm, outtakingFeederVolts, () -> true),
    FULL_OUT(fullOutShooterRPM, fullOutShooterRPM, fullOutFeederVolts, () -> true),
    FULL_IN(fullInShooterRPM, fullInShooterRPM, fullInFeederVolts, () -> true),
    AMP_SHOT(ampShotShooterRPM, ampShotShooterRPM, ampShotFeederVolts, () -> true),
    AUTO_SHOT_NonAmpSide_1(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    AUTO_SHOT_NonAmpSide_2(
        podiumShotShooterRpm,
        podiumShotShooterRpm,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle()),
    FORCE_MANUAL_CONTROL(
        fenderShotShooterRpm,
        fenderShotShooterRpm,
        () -> Robot.operator.getLeftY() * 12, // [-1, 1] * 12V
        () -> true),
    //PRE_SPIN(preSpinRPM, preSpinRPM, () -> 0, () -> true),
    //OFF(() -> 0, () -> 0, () -> 0, () -> true),
    //OUTTAKE_BACKWARDS(() -> -4000, () -> -4000, () -> -5, () -> true),
    CLEANING(() -> 10, () -> 10, () -> 1, () -> true),
    FEEDER_SHOT(
        feederShotRPM,
        feederShotRPM,
        fenderShotFeederVolts,
        () -> Robot.shooterPivot.isAtTargetAngle());
    private final DoubleSupplier leftRpm, rightRpm, feederRpm;
    private final BooleanSupplier additionalFeederCondition;
  }*/

  @RequiredArgsConstructor
  public enum FeederState {
    FENDER_SHOT(fenderShotFeederVolts, () -> Robot.shooterPivot.isAtTargetAngle()),
    PODIUM_SHOT(podiumShotFeederVolts, () -> Robot.shooterPivot.isAtTargetAngle()),
    ELEVATOR_SHOT(elevatorShotFeederVolts, () -> Robot.shooterPivot.isAtTargetAngle()),
    HOLDING_GP(holdingFeederVolts, () -> true),
    INTAKING(intakingFeederVolts, () -> true),
    OUTTAKE_FORWARD(outtakingFeederVolts, () -> true),
    FULL_OUT(fullOutFeederVolts, () -> true),
    FULL_IN(fullInFeederVolts, () -> true),
    AMP_SHOT(ampShotFeederVolts, () -> true),
    FORCE_MANUAL_CONTROL(
        () -> Robot.operator.getLeftY() * 12, // [-1, 1] * 12V
        () -> true),
    OFF(() -> 0, () -> true),
    OUTTAKE_BACKWARDS(() -> -5, () -> true),
    CLEANING(() -> 1, () -> true),
    FEEDER_SHOT(fenderShotFeederVolts, () -> Robot.shooterPivot.isAtTargetAngle());
    private final DoubleSupplier feederRpm;
    private final BooleanSupplier additionalFeederCondition;
  }

  @RequiredArgsConstructor
  public enum FlywheelState {
    PRE_SPIN(preSpinRPM, preSpinRPM),
    OFF(() -> 0, () -> 0),
    OUTTAKE_BACKWARDS(() -> -4000, () -> -4000),
    FENDER_SHOT(() -> 4000, () -> 4000),
    PODIUM_SHOT(() -> 4000, () -> 4000),
    AMP_SHOT(() -> 4000, () -> 4000),
    OUTTAKE_FORWARD(() -> 4000, () -> 4000),
    ELEVATOR_SHOT(() -> 4000, () -> 4000);
    private final DoubleSupplier leftRpm, rightRpm;
  }

  /*@Setter
  @Getter*/
  @AutoLogOutput(key = "Shooter/FeederState")
  @Setter
  @Getter
  public FeederState feederState = FeederState.OFF;
  // public State state = State.OFF;
  // @AutoLogOutput(key = "Shooter/flyWheelState")
  @AutoLogOutput(key = "Shooter/FlywheelState")
  @Setter
  @Getter
  public FlywheelState flywheelState = FlywheelState.OFF;

  private final ShooterIO IO;
  public final ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs, feederState, flywheelState);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs, feederState, flywheelState);

    boolean shouldSpinFeeder = debouncer.calculate(isAtTarget());
    Logger.recordOutput("Shooter/Should spin feeder", shouldSpinFeeder);

    if (feederState == FeederState.INTAKING && hasGamePiece()) {
      feederState = FeederState.HOLDING_GP;
    }

    if ((shouldSpinFeeder && feederState.additionalFeederCondition.getAsBoolean())
        || feederState == Shooter.FeederState.FORCE_MANUAL_CONTROL) {
      IO.setFeederVolts(feederState.feederRpm.getAsDouble());
    } else {
      IO.setFeederVolts(0.0);
    }

    // if (state == State.FENDER_SHOT && !debouncer.calculate(hasGamePiece())) {
    // state = State.OFF;
    // }

    double differential = shooterDifferentialRpm.getAsDouble();

    // if (state == State.FEEDING) {
    //   differential = 0;
    // }

    if (feederState == FeederState.OFF) {
      IO.setShooterVolts(0, 0);
    } else {
      IO.setMotorSetPoint(
          flywheelState.leftRpm.getAsDouble() + differential,
          flywheelState.rightRpm.getAsDouble() - differential);
    }
    Logger.processInputs("Shooter", inputs);
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

    double differential = shooterDifferentialRpm.getAsDouble();

    // if (state == State.FEEDING) {
    //   differential = 0;
    // }

    return Math.abs(flywheelState.leftRpm.getAsDouble() + differential - inputs.leftSpeedRPM)
            < atGoalThresholdRPM.get()
        && Math.abs(flywheelState.rightRpm.getAsDouble() - differential - inputs.rightSpeedRPM)
            < atGoalThresholdRPM.get();
  }

  @AutoLogOutput(key = "Shooter/hasGamePiece")
  public boolean hasGamePiece() {
    return (inputs.laserCanDistanceMM < 65);
  }

  public static class Commands {
    /*public static Command setState(State mode) {
      return new InstantCommand(() -> Robot.shooter.setState(mode));
    }*/

    public static Command setFlywheelState(FlywheelState mode) {
      return new InstantCommand(() -> Robot.shooter.setFlywheelState(mode));
    }

    public static Command setFeederState(FeederState mode) {
      return new InstantCommand(() -> Robot.shooter.setFeederState(mode));
    }
  }
}
