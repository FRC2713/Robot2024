package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.VehicleState;
import frc.robot.commands.Cmds;
import frc.robot.commands.otf.RotateScore;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
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
      new LoggedTunableNumber("ShooterPivot/Amp Shot Angle Degrees", 20);
  private static final LoggedTunableNumber directAmpShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Direct Amp Shot Angle Degrees", -20);
  private static final LoggedTunableNumber autoShotOneAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Auto Shot 1 Angle Degrees", 20);
  private static final LoggedTunableNumber elevatorShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Elevator Angle Degrees", 20);
  private static final LoggedTunableNumber feederShotAngleDegrees =
      new LoggedTunableNumber("ShooterPivot/Feeder Shot Angle Degrees", 10);

  private static final LoggedTunableNumber atGoalThresholdDegrees =
      new LoggedTunableNumber("ShooterPivot/At Goal Threshold Degrees", 1);

  private static final LoggedTunableNumber clutchAuto1Degs =
      new LoggedTunableNumber("ShooterPivot/Autos/clutchAuto1Degs", 21);
  private static final LoggedTunableNumber clutchAuto2Degs =
      new LoggedTunableNumber("ShooterPivot/Autos/clutchAuto2Degs", 20);
  private static final LoggedTunableNumber clutchAuto3Degs =
      new LoggedTunableNumber("ShooterPivot/Autos/clutchAuto3Degs", 20);

  private static final LoggedTunableNumber lobShotDegs =
      new LoggedTunableNumber("ShooterPivot/Lob Shot Degs", intakingAngleDegrees.get());

  private static final LoggedTunableNumber prepClimbAngle =
      new LoggedTunableNumber("ShooterPivot/Prep Climb Degrees", 0);

  private static final LoggedTunableNumber onChainAngle =
      new LoggedTunableNumber("ShooterPivot/On Chain Degrees", 10);

  @RequiredArgsConstructor
  public enum State {
    INTAKING(intakingAngleDegrees), // also for outtaking
    FENDER_SHOT(fenderShotAngleDegrees),
    PODIUM_SHOT(podiumShotAngleDegrees),
    ELEVATOR_SHOT(elevatorShotAngleDegrees),
    POSE_AIM_ELEVATOR_SHOT(
        () -> RotateScore.getElevatorOptimalShooterAngle(Robot.swerveDrive.getEstimatedPose())),
    DYNAMIC_AIM(() -> VehicleState.getInstance().getDynamicPivotAngle().getDegrees()),
    POSE_AIM(() -> RotateScore.getOptimalShooterAngle(Robot.swerveDrive.getEstimatedPose())),
    AMP_SHOT(ampShotAngleDegrees),
    DIRECT_AMP_SHOT(directAmpShotAngleDegrees),
    AUTO_SHOT_NonAmpSide_1(autoShotOneAngleDegrees),
    AUTO_SHOT_NonAmpSide_2(fenderShotAngleDegrees),
    PREP_FOR_CLIMB(prepClimbAngle),
    FEEDER_SHOT(feederShotAngleDegrees),
    CLEANING(() -> 0),
    CLUTCH_AUTO_1(clutchAuto1Degs),
    CLUTCH_AUTO_2(clutchAuto2Degs),
    CLUTCH_AUTO_3(clutchAuto3Degs),
    OFF(() -> 0),
    LOB_SHOT(lobShotDegs),
    ON_CHAIN_ANGLE(onChainAngle);

    private final DoubleSupplier pivotAngleDegrees;
  }

  @Getter
  @Setter
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

  public double getLeftPosition() {
    return inputs.angleDegreesLeft;
  }

  public double getRightPosition() {
    return inputs.angleDegreesRight;
  }

  @AutoLogOutput(key = "ShooterPivot/isAtTargetAngle")
  public boolean isAtTargetAngle() {
    return (Math.abs(inputs.angleDegreesLeft - this.state.pivotAngleDegrees.getAsDouble())
            < atGoalThresholdDegrees.get())
        && (Math.abs(inputs.angleDegreesRight - this.state.pivotAngleDegrees.getAsDouble())
            < atGoalThresholdDegrees.get());
  }

  public static class Commands {
    public static Command setModeAndWait(State mode) {
      return Cmds.setState(mode)
          .andThen(new WaitUntilCommand(() -> (Robot.shooterPivot.isAtTargetAngle())));
    }
  }
}
