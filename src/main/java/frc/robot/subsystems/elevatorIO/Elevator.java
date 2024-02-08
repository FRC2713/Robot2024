package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AccelerationCalc;
import frc.robot.util.LoggableMotor;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SuperStructureBuilder;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /*public final ElevatorIO IO;

  private final ProfiledPIDController elevatorController;
  private final ElevatorInputsAutoLogged inputs;
  private double targetHeight = 0;*/

  @Getter private final ElevatorInputsAutoLogged inputs;
  @Getter private final ElevatorIO IO;
  @Getter private double targetHeight = 0.0;
  public boolean manualControl = false;
  @Getter private final ElevatorFeedforward feedforward;
  @Getter private LoggableMotor leftMotor, rightMotor;
  @Getter private AccelerationCalc accelCalc;

  public Elevator(ElevatorIO IO) {
    this.IO = IO;

    this.feedforward = Constants.ElevatorConstants.ELEVATOR_GAINS.createElevatorFeedforward();
    this.inputs = new ElevatorInputsAutoLogged();
    this.IO.updateInputs(inputs);

    this.leftMotor = new LoggableMotor("Elevator Left", DCMotor.getNEO(1));
    this.rightMotor = new LoggableMotor("Elevator Right", DCMotor.getNEO(1));
    this.accelCalc = new AccelerationCalc(5);
  }

  public void setTargetHeight(double targetHeightInches) {
    this.targetHeight = targetHeightInches;
    if (targetHeightInches > Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS)) {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight =
          MathUtil.clamp(
              targetHeightInches,
              0,
              Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS));
    }
    IO.setTargetHeight(this.targetHeight);
  }

  public void resetController() {
    IO.reset();
  }

  @Override
  public void periodic() {

    IO.updateInputs(inputs);
    // Logger.recordOutput("Elevator/Setpoint Velocity", state.velocity);
    Logger.recordOutput("Elevator/Setpoint Position", this.targetHeight);
    Logger.recordOutput("Elevator/isAtTarget", atTargetHeight());
    Logger.processInputs("Elevator", inputs);
  }

  public boolean atTargetHeight() {
    return Math.abs(getCurrentHeight() - targetHeight) < 1;
  }

  public double getCurrentHeight() {
    return inputs.heightInchesRight;
  }

  public static class Commands {

    public static Command setToHeightAndWait(double targetHeightInches) {
      return new SequentialCommandGroup(
          setToHeight(targetHeightInches),
          new WaitUntilCommand(() -> Robot.elevator.atTargetHeight()));
    }

    public static Command setToHeight(double height) {
      return new InstantCommand(() -> Robot.elevator.setTargetHeight(height), Robot.elevator);
    }

    public static Command toHeight(SuperStructureBuilder structure) {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(structure.getElevatorHeight()));
    }
  }
}
