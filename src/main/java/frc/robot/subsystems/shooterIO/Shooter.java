package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber fenderShotShooterRpm =
      new LoggedTunableNumber("Flywheel/Fender Shot RPM", 3500);
  private static final LoggedTunableNumber fenderShotFeederVolts =
      new LoggedTunableNumber("Flywheel/Fender Shot Feeder Volts", 10);

  private static final LoggedTunableNumber holdingGpShooterRpm =
      new LoggedTunableNumber("Flywheel/Resting RPM", 0);
  private static final LoggedTunableNumber holdingFeederVolts =
      new LoggedTunableNumber("Flywheel/Resting Feeder Volts", 0);

  private static final LoggedTunableNumber intakingShooterRpm =
      new LoggedTunableNumber("Flywheel/Intaking Feeder RPM", 0);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Flywheel/Intaking Feeder Volts", 10);

  private static final LoggedTunableNumber atGoalThresholdRPM =
      new LoggedTunableNumber("Flywheel/At Goal Threshold RPM", 50);

  private static final double WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE = 1.0;
  private final Debouncer debouncer =
      new Debouncer(WAIT_TIME_AFTER_SHOT_TO_TRANSITION_STATE, DebounceType.kBoth);

  @RequiredArgsConstructor
  public enum State {
    FENDER_SHOT(fenderShotShooterRpm, fenderShotShooterRpm, fenderShotFeederVolts),
    HOLDING_GP(holdingGpShooterRpm, holdingGpShooterRpm, holdingFeederVolts),
    INTAKING(intakingShooterRpm, intakingShooterRpm, intakingFeederVolts),
    OFF(() -> 0, () -> 0, () -> 0);
    private final DoubleSupplier leftRpm, rightRpm, feederRpm;
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

    if (!isAtTarget()) {
      IO.setFeederVolts(0.0);
    } else {
      IO.setFeederVolts(state.feederRpm.getAsDouble());
    }

    if (state == State.INTAKING && hasGamePiece()) {
      state = State.HOLDING_GP;
    }

    if (state == State.FENDER_SHOT && !debouncer.calculate(hasGamePiece())) {
      state = State.OFF;
    }

    IO.setMotorSetPoint(state.leftRpm.getAsDouble(), state.rightRpm.getAsDouble());
    Logger.processInputs("Shooter", inputs);
  }

  @AutoLogOutput(key = "Flywheel/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(inputs.leftSpeedRPM - state.leftRpm.getAsDouble())
            < atGoalThresholdRPM.getAsDouble()
        && Math.abs(inputs.rightSpeedRPM - state.rightRpm.getAsDouble())
            < atGoalThresholdRPM.getAsDouble();
  }

  @AutoLogOutput(key = "Flywheel/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.sensorVoltage >= FeederConstants.SENSOR_THRESHOLD;
  }

  public static class Commands {
    public static Command setState(State mode) {
      return new InstantCommand(() -> Robot.shooter.setState(mode));
    }
  }
}
