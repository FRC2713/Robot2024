package frc.robot.subsystems.feederIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  DCMotorSim sim =
      new DCMotorSim(
          DCMotor.getNEO(1), Constants.FeederConstants.GERING, Constants.FeederConstants.MOI);
  private double volts;

  public FeederIOSim() {}

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    sim.setInputVoltage(volts);
  }
  // @Override
  // public boolean atTarget() {
  //     return Math.abs(sim.getAngularVelocityRPM() -) > 0.01;
  // }
  @Override
  public void updateInputs(FeederInputsAutoLogged inputs) {
    sim.update(0.02);
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.outputVoltage = volts;
    inputs.isOn = Math.abs(sim.getAngularVelocityRPM()) > 0.05;
    inputs.tempCelcius = 0;
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.positionDeg = Units.rotationsToDegrees(sim.getAngularPositionRotations());
  }
}
