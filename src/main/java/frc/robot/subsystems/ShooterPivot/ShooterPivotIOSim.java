package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ShooterPivotIOSim implements ShooterPivotIO {

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Constants.ShooterPivotConstants.GEARING,
          SingleJointedArmSim.estimateMOI(
              Constants.ShooterPivotConstants.LENGTH_METERS,
              Constants.ShooterPivotConstants.MASS_KG),
          Constants.ShooterPivotConstants.LENGTH_METERS,
          Units.degreesToRadians(Constants.ShooterPivotConstants.MAX_ANGLE_DEGREES - 10),
          Units.degreesToRadians(Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES + 10),
          Constants.ShooterPivotConstants.SIMULATE_GRAVITY,
          Constants.ShooterPivotConstants.STARTING_ANGLE_RADS);

  public ShooterPivotIOSim() {}

  // TODO
  @Override
  public void setCurrentLimit(int currentLimit) {}

  @Override
  public void setPosition(double angleDeg) {
    sim.setState(VecBuilder.fill(Units.degreesToRadians(angleDeg), 0.0));
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void reseed(double absoluteEncoderVolts) {
    sim.setState(
        VecBuilder.fill(
            Units.degreesToRadians(Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES), 0.0));
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }

    sim.update(0.02);

    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);

    inputs.angleDegreesOne = Units.radiansToDegrees(sim.getAngleRads()) + (Math.random() * 5 - 2.5);
    // inputs.angleDegreesTwo = Units.radiansToDegrees(sim.getAngleRads());
    inputs.angleDegreesRange = 0.0;

    inputs.absoluteEncoderAdjustedAngle = Units.radiansToDegrees(sim.getAngleRads());

    inputs.velocityDegreesPerSecondOne = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.velocityDegreesPerSecondTwo = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.velocityDegreesPerSecondRange = 0.0;

    inputs.tempCelciusOne = 0.0;
    inputs.tempCelciusTwo = 0.0;

    inputs.currentDrawOne = sim.getCurrentDrawAmps();
    inputs.currentDrawTwo = sim.getCurrentDrawAmps();

    inputs.limSwitch =
        Units.radiansToDegrees(sim.getAngleRads())
            >= Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES;
  }
}
