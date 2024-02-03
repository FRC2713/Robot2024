package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
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

    simLeft.update(0.02);
    simRight.update(0.02);

    inputs.leftOutputVoltage = simLeft.getOutput(0);
    inputs.leftIsOn = simLeft.getAngularVelocityRPM() > 0.005;
    inputs.leftVelocityRPM = simLeft.getAngularVelocityRPM();
    inputs.leftTempCelcius = 0.0;
    inputs.leftCurrentAmps = simLeft.getCurrentDrawAmps();
    inputs.leftPositionRad = simLeft.getAngularPositionRad();

    inputs.rightOutputVoltage = simRight.getOutput(0);
    inputs.rightIsOn = simRight.getAngularVelocityRPM() > 0.005;
    inputs.rightVelocityRPM = simRight.getAngularVelocityRPM();
    inputs.rightTempCelcius = 0.0;
    inputs.rightCurrentAmps = simRight.getCurrentDrawAmps();
    inputs.rightPositionRad = simRight.getAngularPositionRad();

    inputs.sensorRange = 0.0;
    inputs.sensorStatus = "Short";
  }

  @Override
  public boolean hasGamepiece() {
    if (intakeHasStarted && intakeIsRunning.hasElapsed(numSecToObtain)) {
      intakeIsRunning.stop();

      return true;
    }

    return false;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {

    if (!intakeHasStarted && leftVolts > 0) {
      // start intaking
      intakeIsRunning.restart();
      intakeHasStarted = true;
    }

    if (leftVolts < 0) {
      // outtaking immediately removes the gamepiece
      intakeHasStarted = false;
      intakeIsRunning.stop();
    }

    simLeft.setInputVoltage(leftVolts);
    simRight.setInputVoltage(-rightVolts);

  }
}
