package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.rhr.RHRPIDFFController;

public class ShooterIOSim implements ShooterIO {
  RHRPIDFFController shooterController;

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

  public ShooterIOSim() {
    shooterController = ShooterConstants.SHOOTER_GAINS.createRHRController();
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    leftFlyWheel.update(0.02);
    rightFlyWheel.update(0.02);

    inputs.leftOutputVoltage = this.leftVolts;
    inputs.rightOutputVoltage = this.rightVolts;

    inputs.leftDrawAmp = leftFlyWheel.getCurrentDrawAmps();
    inputs.rightDrawAmp = rightFlyWheel.getCurrentDrawAmps();

    inputs.leftTempCelcius = 0.0;
    inputs.rightTempCelcius = 0.0;

    inputs.rightSpeedRPM = rightFlyWheel.getAngularVelocityRPM();
    inputs.leftSpeedRPM = leftFlyWheel.getAngularVelocityRPM();

    inputs.leftPosDeg = 0.0;
    inputs.rightPosDeg = 0.0;
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

  @Override
  public void setMotorSetPoint(double setpointRPM) {
    shooterController.setSetpoint(setpointRPM);
    var voltage = shooterController.calculate(setpointRPM);
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }
}
