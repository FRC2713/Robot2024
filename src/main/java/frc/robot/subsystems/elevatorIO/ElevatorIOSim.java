package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.rhr.RHRPIDFFController;

public class ElevatorIOSim implements ElevatorIO {
  RHRPIDFFController heightControllerRight;

  public ElevatorIOSim() {
    heightControllerRight = ElevatorConstants.ELEVATOR_GAINS.createRHRController();
  }

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          Constants.ElevatorConstants.GEARING,
          Constants.ElevatorConstants.CARRIAGE_MASS_KG,
          Constants.ElevatorConstants.DRUM_RADIUS_METERS,
          Constants.ElevatorConstants.MIN_HEIGHT_METERS,
          Constants.ElevatorConstants.MAX_HEIGHT_METERS,
          Constants.ElevatorConstants.SIMULATE_GRAVITY,
          Constants.ElevatorConstants.STARTING_HEIGHT_METERS);

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double desiredVoltage = heightControllerRight.calculate(inputs.heightInchesRight);
    desiredVoltage = MathUtil.clamp(desiredVoltage, -12, 12);
    if (DriverStation.isDisabled()) {
      desiredVoltage = 0.;
    }
    sim.setInputVoltage(desiredVoltage);

    sim.update(0.02);
    inputs.outputVoltageLeft = desiredVoltage;
    inputs.heightInchesLeft = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondLeft = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps() / 2.0;
    inputs.outputVoltageRight = desiredVoltage;
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps() / 2.0;
  }

  @Override
  public void reset() {
    heightControllerRight.reset();
  }

  @Override
  public boolean shouldApplyFF() {
    return true;
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setTargetHeight(double heightInches) {
    heightControllerRight.setSetpoint(heightInches);
  }

  @Override
  public void setCurrentLimits() {
    // TODO
    return;
  }
}
