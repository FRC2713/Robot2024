package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

  private static final FlywheelSim leftFlyWheel =
      new FlywheelSim(
          DCMotor.getNeoVortex(1),
          Constants.ShooterConstants.GEARING,
          Constants.ShooterConstants.MOI);

  private static final FlywheelSim rightFlyWheel =
      new FlywheelSim(
          DCMotor.getNeoVortex(1),
          Constants.ShooterConstants.GEARING,
          Constants.ShooterConstants.MOI);

  private double leftVolts;

  private double rightVolts;

  @Override
  public void updateInputs(ShooterInputs inputs) {
    // setLeftVoltage(MathUtil.clamp(0, 0, 0));
    leftFlyWheel.update(0.02);
    rightFlyWheel.update(0.02);

    // inputs.leftOutputVoltage = MathUtil.clamp(leftFlyWheel.getOutput(0), -12, 12);
    // inputs.rightOutputVoltage = MathUtil.clamp(rightFlyWheel.getOutput(0), -12, 12);
    inputs.leftOutputVoltage = this.leftVolts;
    inputs.rightOutputVoltage = this.rightVolts;

    inputs.leftFLyWheelDrawAmp = leftFlyWheel.getCurrentDrawAmps();
    inputs.rightFLyWheelDrawAmp = rightFlyWheel.getCurrentDrawAmps();

    inputs.leftTempCelcius = 0.0;
    inputs.rightTempCelcius = 0.0;

    inputs.rightFlyWheelSpeedRPM = rightFlyWheel.getAngularVelocityRPM();
    inputs.leftFlyWheelSpeedRPM = leftFlyWheel.getAngularVelocityRPM();
  }

  @Override
  public void setLeftVoltage(double voltage) {
    leftFlyWheel.setInputVoltage(voltage);
    this.leftVolts = voltage;
  }

  @Override
  public void setRightVoltage(double voltage) {
    rightFlyWheel.setInputVoltage(voltage);
    this.rightVolts = voltage;
  }
}
