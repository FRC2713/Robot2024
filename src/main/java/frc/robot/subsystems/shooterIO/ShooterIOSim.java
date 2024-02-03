package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.MathUtil;
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

  @Override
  public void updateInputs(ShooterInputs inputs) {
    // setLeftVoltage(MathUtil.clamp(0, 0, 0));
    leftFlyWheel.update(0.02);
    rightFlyWheel.update(0.02);

    inputs.leftOutputVoltage = MathUtil.clamp(leftFlyWheel.getOutput(0), -12, 12);
    inputs.rightOutputVoltage = MathUtil.clamp(rightFlyWheel.getOutput(0), -12, 12);

    inputs.leftFLyWheelDrawAmp = leftFlyWheel.getCurrentDrawAmps();
    inputs.rightFLyWheelDrawAmp = rightFlyWheel.getCurrentDrawAmps();

    inputs.leftTempCelcius = 0.0;
    inputs.rightTempCelcius = 0.0;
  }

  @Override
  public void setLeftVoltage(double voltage) {
    leftFlyWheel.setInputVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    rightFlyWheel.setInputVoltage(voltage);
  }

  @Override
  public void setLeftMotorRPMSetPoint(double rPM)
  {

  }

  @Override
  public void setRightMotorRPMSetPoint(double rPM)
  {

  }

  @Override
  public void effort(ShooterInputs inputs)
  {

  }
}
