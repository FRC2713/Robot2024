package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.rhr.RHRFeedForward;
import frc.robot.rhr.RHRPIDFFController;

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
          !Constants.ShooterPivotConstants.SIMULATE_GRAVITY,
          Constants.ShooterPivotConstants.STARTING_ANGLE_RADS);
  private double targetAngle;

  private RHRPIDFFController motorController;

  private RHRFeedForward feedforward;

  private double voltage;

  public ShooterPivotIOSim() {
    motorController = Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createRHRController();
    feedforward = Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createRHRFeedForward();
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }

    sim.update(0.02);

    inputs.outputVoltage = this.voltage;

    inputs.angleDegreesOne = Units.radiansToDegrees(sim.getAngleRads()) + (Math.random() * 5 - 2.5);

    inputs.absoluteEncoderAdjustedAngle = Units.radiansToDegrees(sim.getAngleRads());

    inputs.velocityDegreesPerSecondOne = Units.radiansToDegrees(sim.getVelocityRadPerSec());

    inputs.tempCelciusOne = 0.0;

    inputs.currentDrawOne = sim.getCurrentDrawAmps();

    double effort = motorController.calculate(inputs.absoluteEncoderAdjustedAngle, targetAngle);
    effort = MathUtil.clamp(effort, -12, 12);
    setVoltage(effort);
  }

  @Override
  public void reseedPosition(double angleDeg) {
    sim.setState(VecBuilder.fill(Units.degreesToRadians(angleDeg), 0.0));
  }

  @Override
  public void setTargetPosition(double angleDeg) {
    this.targetAngle = angleDeg;
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
    this.voltage = volts;
  }
}
