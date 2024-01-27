package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.rhr.RHRPIDFFController;

public class ElevatorIOSim implements ElevatorIO {
  RHRPIDFFController heightcontrollerleft;
  RHRPIDFFController heightcontrollerright;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          Constants.ElevatorConstants.GEARING,
          Constants.ElevatorConstants.CARRIAGE_MASS_KG,
          Constants.ElevatorConstants.DRUM_RADIUS_METERS,
          Constants.ElevatorConstants.MIN_HEIGHT_METERS,
          Constants.ElevatorConstants.MAX_HEIGHT_METERS,
          Constants.ElevatorConstants.SIMULATE_GRAVITY,
          Constants.ElevatorConstants.STARTING_HEIGHT_METERS);

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double desiredvoltageleft = heightcontrollerleft.calculate(inputs.heightInchesLeft);
    double desiredvoltageright = heightcontrollerright.calculate(inputs.heightInchesRight);
    sim.setInputVoltage(desiredvoltageright);
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
      // System.out.print("hello");
    }
    sim.update(0.02);
    inputs.outputVoltageLeft = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInchesLeft = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondLeft = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps() / 2.0;
    inputs.outputVoltageRight = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    // System.out.println(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps() / 2.0;
  }

  @Override
  public void reset() {
    heightcontrollerright.reset();
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
    heightcontrollerright.setSetpoint(heightInches);
    heightcontrollerleft.setSetpoint(heightInches);
    //left is un-utilized
  }

  
}
