package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim simLeftRollers =
      new DCMotorSim(
          DCMotor.getNEO(1), Constants.IntakeConstants.LEFT_GEARING, Constants.IntakeConstants.MOI);

  private final DCMotorSim simRightRollers =
      new DCMotorSim(
          DCMotor.getNEO(1),
          Constants.IntakeConstants.RIGHT_GEARING,
          Constants.IntakeConstants.MOI);

  private Timer intakeIsRunning = new Timer();
  private boolean intakeHasStarted = false;
  private final double numSecToObtain = 5;

  public IntakeIOSim() {
    intakeIsRunning.start();
    intakeIsRunning.stop(); // just so that I can use restart from the get-go
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    // TODO Auto-generated method stub
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.leftOutputVoltage = simLeftRollers.getOutput(0);
    inputs.leftIsOn = simLeftRollers.getAngularVelocityRPM() > 0.005;
    inputs.leftVelocityRPM = simLeftRollers.getAngularVelocityRPM();
    inputs.leftTempCelcius = 0.0;
    inputs.leftCurrentAmps = simLeftRollers.getCurrentDrawAmps();
    inputs.leftPositionRad = simLeftRollers.getAngularPositionRad();

    inputs.rightOutputVoltage = simRightRollers.getOutput(0);
    inputs.rightIsOn = simRightRollers.getAngularVelocityRPM() > 0.005;
    inputs.rightVelocityRPM = simRightRollers.getAngularVelocityRPM();
    inputs.rightTempCelcius = 0.0;
    inputs.rightCurrentAmps = simRightRollers.getCurrentDrawAmps();
    inputs.rightPositionRad = simRightRollers.getAngularPositionRad();

    inputs.sensorRange = 0.0;
    inputs.sensorStatus = "Short";
  }

  @Override
  public boolean hasGamepiece() {
    if (intakeHasStarted && intakeIsRunning.hasElapsed(numSecToObtain)) {
      intakeIsRunning.stop();
      intakeHasStarted = false;
      return true;
    }

    return false;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    if (!intakeHasStarted && leftVolts > 0 || rightVolts > 0) {
      intakeIsRunning.restart();
      intakeHasStarted = true;
    }

    simLeftRollers.setInputVoltage(leftVolts);
    simRightRollers.setInputVoltage(rightVolts);
  }
}
