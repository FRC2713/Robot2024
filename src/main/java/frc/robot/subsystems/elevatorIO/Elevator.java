package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @Getter private final ProfiledPIDController elevatorController;
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
    System.out.println(Constants.ElevatorConstants.ELEVATOR_GAINS.toString());
    elevatorController =
        Constants.ElevatorConstants.ELEVATOR_GAINS.createProfiledPIDController(
            new Constraints(100, 200));
    SmartDashboard.putData("Elevator PID", elevatorController);
    this.leftMotor = new LoggableMotor("Elevator Left", DCMotor.getNEO(1));
    this.rightMotor = new LoggableMotor("Elevator Right", DCMotor.getNEO(1));
    this.accelCalc = new AccelerationCalc(5);
  }

  public void setTargetHeight(double targetHeightInches) {
    if (targetHeightInches > Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS)) {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight =
          MathUtil.clamp(
              targetHeightInches,
              0,
              Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS));
      return;
    }
    this.targetHeight = targetHeightInches;
  }

  public void resetController() {
    elevatorController.reset(inputs.heightInchesRight, inputs.velocityInchesPerSecondRight);
  }

  @Override
  public void periodic() {

    IO.updateInputs(inputs);
    double effortLeft = elevatorController.calculate(inputs.heightInchesRight, targetHeight);

    State state = elevatorController.getSetpoint();
    if (IO.shouldApplyFF()) {
      effortLeft += feedforward.calculate(state.position, state.velocity);
    }

    if (manualControl) {
      effortLeft = -1 * Robot.operator.getLeftY();
    }

    effortLeft = MathUtil.clamp(effortLeft, -12, 12);
    Logger.recordOutput("Elevator/Setpoint Velocity", state.velocity);
    Logger.recordOutput("Elevator/Setpoint Position", state.position);
    IO.setVoltage(effortLeft);
    Logger.recordOutput("Elevator/isAtTarget", atTargetHeight());
    Logger.recordOutput("Elevator/heightInchesLeft", inputs.heightInchesLeft);
    Logger.recordOutput("Elevator/heightInchesRight", inputs.heightInchesRight);

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
      return setToHeight(targetHeightInches)
          .repeatedly()
          .until(() -> Robot.elevator.atTargetHeight());
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
