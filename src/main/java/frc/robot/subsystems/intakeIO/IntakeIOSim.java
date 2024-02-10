package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim simLeft =
      new DCMotorSim(
          DCMotor.getNEO(1), Constants.IntakeConstants.LEFT_GEARING, Constants.IntakeConstants.MOI);

  private final DCMotorSim simRight =
      new DCMotorSim(
          DCMotor.getNEO(1),
          Constants.IntakeConstants.RIGHT_GEARING,
          Constants.IntakeConstants.MOI);

  private double leftVolts;

  private double rightVolts;

  public IntakeIOSim() {}

  @Override
  public void setCurrentLimit(int currentLimit) {
    // TODO Auto-generated method stub
  }

  @Override
  public void updateInputs(IntakeInputsAutoLogged inputs) {

    simLeft.update(0.02);
    simRight.update(0.02);

    inputs.leftOutputVoltage = MathUtil.clamp(leftVolts, -12, 12);
    inputs.leftIsOn = Math.abs(simLeft.getAngularVelocityRPM()) > 0.005;
    inputs.leftVelocityRPM = simLeft.getAngularVelocityRPM();
    inputs.leftTempCelcius = 0.0;
    inputs.leftCurrentAmps = simLeft.getCurrentDrawAmps();
    inputs.leftPositionRad = simLeft.getAngularPositionRad();

    inputs.rightOutputVoltage = MathUtil.clamp(rightVolts, -12, 12);
    inputs.rightIsOn = Math.abs(simRight.getAngularVelocityRPM()) > 0.005;
    inputs.rightVelocityRPM = simRight.getAngularVelocityRPM();
    inputs.rightTempCelcius = 0.0;
    inputs.rightCurrentAmps = simRight.getCurrentDrawAmps();
    inputs.rightPositionRad = simRight.getAngularPositionRad();

    inputs.sensorRange = 0.0;
    inputs.sensorStatus = "Short";
  }

  @Override
  public boolean hasGamepiece() {
    return false;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    simLeft.setInputVoltage(leftVolts);
    this.leftVolts = leftVolts;
    simRight.setInputVoltage(-rightVolts);
    this.rightVolts = -rightVolts;
  }
}
