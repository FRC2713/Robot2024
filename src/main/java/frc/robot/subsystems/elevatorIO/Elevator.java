package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AccelerationCalc;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  @Getter private final ElevatorInputsAutoLogged inputs;
  @Getter private final ElevatorIO IO;
  public boolean manualControl = false;
  @Getter private final ElevatorFeedforward feedforward;
  @Getter private AccelerationCalc accelCalc;

  private static final LoggedTunableNumber maxHeight =
      new LoggedTunableNumber(
          "Elevator/Max Height",
          Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS));

  private static final LoggedTunableNumber chainApproachHeight =
      new LoggedTunableNumber("Elevator/Chain Approach Height", 10);

  private static final LoggedTunableNumber onChainHeight =
      new LoggedTunableNumber("Elevator/On chain height", 5);

  private static final LoggedTunableNumber ampHeight =
      new LoggedTunableNumber("Elevator/Amp height", 15);

  private static final LoggedTunableNumber elevatorShotHeight =
      new LoggedTunableNumber("Elevator/Amp height", 15);

  @RequiredArgsConstructor
  public enum State {
    MIN_HEIGHT(() -> 0),
    MAX_HEIGHT(maxHeight),
    CHAIN_APPROACH_HEIGHT(chainApproachHeight),
    ON_CHAIN_HEIGHT(onChainHeight),
    AMP(ampHeight),
    ELEVATORSHOT(elevatorShotHeight),
    OFF(() -> 0);

    private final DoubleSupplier height;
  }

  @Setter
  @AutoLogOutput(key = "Elevator/State")
  public State state = State.OFF;

  public Elevator(ElevatorIO IO) {
    this.IO = IO;

    this.feedforward = Constants.ElevatorConstants.ELEVATOR_GAINS.createElevatorFeedforward();
    this.inputs = new ElevatorInputsAutoLogged();
    this.IO.updateInputs(inputs);

    this.accelCalc = new AccelerationCalc(5);
  }

  public void resetController() {
    IO.reset();
  }

  @Override
  public void periodic() {
    IO.updateInputs(inputs);

    if (state == State.OFF) {
      IO.setTargetHeight(inputs.heightInchesLeft);
    } else {
      var target = state.height.getAsDouble();
      Logger.recordOutput("Elevator/Target", target);
      IO.setTargetHeight(target);
    }

    Logger.processInputs("Elevator", inputs);
  }

  @AutoLogOutput(key = "Elevator/isAtTarget")
  public boolean atTargetHeight() {
    return Math.abs(getCurrentHeight() - state.height.getAsDouble()) < 1;
  }

  public double getCurrentHeight() {
    return inputs.heightInchesRight;
  }

  public static class Commands {
    public static Command setState(State state) {
      return new InstantCommand(() -> Robot.elevator.setState(state));
    }
  }
}
