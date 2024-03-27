package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.rhr.RHRPIDFFController;
import frc.robot.util.PIDFFGains;

public class ShooterIOSim implements ShooterIO {
  RHRPIDFFController leftController, rightController;

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

  private static final FlywheelSim feeder = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.0001);

  private double leftVolts, rightVolts, feederVolts;
  Timer fakeGamepieceTimer = new Timer();

  public ShooterIOSim() {
    var simGains = PIDFFGains.builder().kV(0.0017).build();
    leftController = new RHRPIDFFController(simGains);
    rightController = new RHRPIDFFController(simGains);
  }

  @Override
  public void updateInputs(
      ShooterInputsAutoLogged inputs,
      Shooter.ShooterState shooterState,
      Shooter.FeederState feederState) {
    leftFlyWheel.update(0.02);
    rightFlyWheel.update(0.02);
    feeder.update(0.02);

    leftVolts =
        MathUtil.clamp(leftController.calculate(leftFlyWheel.getAngularVelocityRPM()), -12, 12);
    rightVolts =
        MathUtil.clamp(rightController.calculate(rightFlyWheel.getAngularVelocityRPM()), -12, 12);

    leftFlyWheel.setInputVoltage(leftVolts);
    rightFlyWheel.setInputVoltage(rightVolts);
    feeder.setInputVoltage(feederVolts);

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

    inputs.feederOutputVolts = feederVolts;
    inputs.feederVelocityRPM = feeder.getAngularVelocityRPM();
    inputs.feederStatorCurrentAmps = feeder.getCurrentDrawAmps();
    inputs.feederSupplyCurrentAmps = feeder.getCurrentDrawAmps();

    inputs.LSTripped = false;

    if (feederState == Shooter.FeederState.INTAKE) {
      fakeGamepieceTimer.start();

      // if (fakeGamepieceTimer.get() > 1.0 || inputs.sensorVoltage != 0) {
      //   inputs.sensorVoltage = 4.5;
      // } else {
      //   inputs.sensorVoltage = 0.0;
      // }
    }

    if (feederState == Shooter.FeederState.HOLDING_GP) {
      fakeGamepieceTimer.stop();
      fakeGamepieceTimer.reset();
      // inputs.sensorVoltage = 4.5;
    }

    if (shooterState == Shooter.ShooterState.NO_DIFFERENTIAL_SHOT) {
      fakeGamepieceTimer.start();

      if (fakeGamepieceTimer.get() <= 0.5) {
        // inputs.sensorVoltage = 4.5;
      } else {
        // inputs.sensorVoltage = 0;
      }
    }
  }

  @Override
  public void setMotorSetPoint(double leftRPM, double rightRPM) {
    leftController.setSetpoint(leftRPM);
    rightController.setSetpoint(rightRPM);
  }

  @Override
  public void setFeederVolts(double volts) {
    this.feederVolts = volts;
  }

  @Override
  public void setShooterVolts(double lVolts, double rVolts) {}
}
