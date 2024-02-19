package frc.robot.subsystems.shooterIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber fenderShotRpm =
      new LoggedTunableNumber("Flywheel/Fender Shot RPM", 3500);
  private static final LoggedTunableNumber restingRpm =
      new LoggedTunableNumber("Flywheel/Resting RPM", 500);

  @RequiredArgsConstructor
  public enum State {
    FENDER_SHOT(fenderShotRpm, fenderShotRpm),
    RESTING(restingRpm, restingRpm),
    OFF(() -> 0, () -> 0);
    private final DoubleSupplier leftGoal, rightGoal;
  }

  @Setter public State motionMode = State.OFF;

  private final ShooterIO IO;
  private final ShooterInputsAutoLogged inputs;

  private double targetRPM;

  public Shooter(ShooterIO IO) {
    this.IO = IO;
    this.inputs = new ShooterInputsAutoLogged();
    this.IO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);
    IO.setMotorSetPoint(motionMode.leftGoal.getAsDouble(), motionMode.rightGoal.getAsDouble());

    Logger.recordOutput("Shooter/Mode", motionMode);
    Logger.processInputs("Shooter", inputs);
  }

  @AutoLogOutput(key = "Flywheel/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(inputs.leftSpeedRPM - targetRPM) < 90;
  }

  public static class Commands {
    public static Command setState(State mode) {
      return new InstantCommand(() -> Robot.shooter.setMotionMode(mode));
    }
  }
}
