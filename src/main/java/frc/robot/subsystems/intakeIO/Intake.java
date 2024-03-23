package frc.robot.subsystems.intakeIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber intakeBottomVolts =
      new LoggedTunableNumber("Intake/Intaking Bottom Volts", 10);
  private static final LoggedTunableNumber intakeTopVolts =
      new LoggedTunableNumber("Intake/Intaking Top Volts", 10);

  private static final LoggedTunableNumber outtakeBottomVolts =
      new LoggedTunableNumber("Intake/Outtaking Bottom Volts", -12);
  private static final LoggedTunableNumber outtakeTopVolts =
      new LoggedTunableNumber("Intake/Outtaking Top Volts", -12);

  private static final LoggedTunableNumber fullInBottomVolts =
      new LoggedTunableNumber("Intake/Full-In Top Volts", 12);
  private static final LoggedTunableNumber fullInTopVolts =
      new LoggedTunableNumber("Intake/Full-In Bottom Volts", 12);

  private static final LoggedTunableNumber fullOutBottomVolts =
      new LoggedTunableNumber("Intake/Full-Out Top Volts", -12);
  private static final LoggedTunableNumber fullOutTopVolts =
      new LoggedTunableNumber("Intake/Full-Out Bottom Volts", -12);

  @RequiredArgsConstructor
  public enum State {
    INTAKE_GP(intakeBottomVolts, intakeTopVolts),
    OUTAKE_GP(outtakeBottomVolts, outtakeTopVolts),
    FULL_IN(fullInBottomVolts, fullInTopVolts),
    FULL_OUT(fullOutBottomVolts, fullOutTopVolts),
    NOTE_IN_CHASSIS(() -> -12, () -> 12),
    OFF(() -> 0, () -> 0),
    CLEANING(() -> 1, () -> 1);
    private final DoubleSupplier bottomVolts, topVolts;
  }

  @Setter
  @Getter
  @AutoLogOutput(key = "Intake/State")
  public State state = State.OFF;

  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    IO.setVoltage(state.bottomVolts.getAsDouble(), state.topVolts.getAsDouble());
  }

  public static class Commands {}
}
