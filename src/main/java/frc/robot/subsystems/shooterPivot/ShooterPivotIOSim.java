package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivotIOSim implements ShooterPivotIO {

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Constants.ShooterPivotConstants.GEARING,
          SingleJointedArmSim.estimateMOI(
              Constants.ShooterPivotConstants.LENGTH_METERS,
              Constants.ShooterPivotConstants.MASS_KG),
          Constants.ShooterPivotConstants.LENGTH_METERS,
          Units.degreesToRadians(Constants.ShooterPivotConstants.RETRACTED_ANGLE_DEGREES),
          Units.degreesToRadians(Constants.ShooterPivotConstants.MAX_ANGLE_DEGREES),
          Constants.ShooterPivotConstants.SIMULATE_GRAVITY,
          Constants.ShooterPivotConstants.STARTING_ANGLE_RADS);
  private double targetAngle;
  private double armFeedForwardVolts;

  private PIDController simController;

  private double voltage;

  public ShooterPivotIOSim() {
    simController =
        new PIDController(
            ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKP(),
            ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKI(),
            ShooterPivotConstants.SHOOTER_PIVOT_GAINS.getKD());
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
      return;
    }

    double effort =
        simController.calculate(inputs.absoluteEncoderAdjustedAngle, targetAngle)
            + armFeedForwardVolts;
    effort = MathUtil.clamp(effort, -12, 12);
    this.voltage = effort;

    sim.setInputVoltage(voltage);
    sim.update(0.02);

    inputs.outputVoltageLeft = this.voltage;
    inputs.angleDegreesLeft = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDegreesPerSecondLeft = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps();

    inputs.outputVoltageRight = inputs.outputVoltageLeft;
    inputs.angleDegreesRight = inputs.angleDegreesLeft;
    inputs.velocityDegreesPerSecondRight = inputs.velocityDegreesPerSecondLeft;
    inputs.currentDrawAmpsRight = inputs.currentDrawAmpsLeft;

    inputs.absoluteEncoderAdjustedAngle = Units.radiansToDegrees(sim.getAngleRads());
  }

  @Override
  public void setTargetAngle(double degrees, double arbFeedForward) {
    this.armFeedForwardVolts = arbFeedForward;
    targetAngle = degrees;
  }
}
