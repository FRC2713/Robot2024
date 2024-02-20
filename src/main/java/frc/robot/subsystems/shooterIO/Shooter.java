package frc.robot.subsystems.shooterIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
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

  private static final LoggedTunableNumber restingShooterRpm =
      new LoggedTunableNumber("Flywheel/Resting RPM", 0);
  private static final LoggedTunableNumber restingFeederVolts =
      new LoggedTunableNumber("Flywheel/Resting Feeder Volts", 0);

  private static final LoggedTunableNumber intakingShooterRpm =
      new LoggedTunableNumber("Flywheel/Intaking Feeder RPM", 0);
  private static final LoggedTunableNumber intakingFeederVolts =
      new LoggedTunableNumber("Flywheel/Intaking Feeder Volts", 10);

  private static final LoggedTunableNumber atGoalThresholdRPM =
      new LoggedTunableNumber("Flywheel/At Goal Threshold RPM", 50);

  @RequiredArgsConstructor
  public enum State {
    FENDER_SHOT(fenderShotShooterRpm, fenderShotShooterRpm, fenderShotFeederVolts),
    RESTING(restingShooterRpm, restingShooterRpm, restingFeederVolts),
    INTAKING(intakingShooterRpm, intakingShooterRpm, intakingFeederVolts),
    OFF(() -> 0, () -> 0, () -> 0);
    private final DoubleSupplier leftRpm, rightRpm, feederRpm;
  }

  @Setter public State motionMode = State.OFF;

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);
    IO.setMotorSetPoint(motionMode.leftRpm.getAsDouble(), motionMode.rightRpm.getAsDouble());

    if (!isAtTarget()) {
      IO.setFeederVolts(0.0);
    } else {
      IO.setFeederVolts(motionMode.feederRpm.getAsDouble());
    }

    Logger.recordOutput("Shooter/Mode", motionMode);
    Logger.processInputs("Shooter", inputs);
  }

  @AutoLogOutput(key = "Flywheel/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(inputs.leftSpeedRPM - motionMode.leftRpm.getAsDouble())
            < atGoalThresholdRPM.getAsDouble()
        && Math.abs(inputs.rightSpeedRPM - motionMode.rightRpm.getAsDouble())
            < atGoalThresholdRPM.getAsDouble();
  }

  @AutoLogOutput(key = "Flywheel/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.sensorVoltage >= FeederConstants.SENSOR_THRESHOLD;
  }

  public static class Commands {
    public static Command setState(State mode) {
      return new InstantCommand(() -> Robot.shooter.setMotionMode(mode));
    }
  }
}
