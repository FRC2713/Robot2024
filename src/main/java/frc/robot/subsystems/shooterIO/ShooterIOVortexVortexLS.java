package frc.robot.subsystems.shooterIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class ShooterIOVortexVortexLS implements ShooterIO {
  private final CANSparkFlex leftMotor =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);
  private final CANSparkFlex rightMotor =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_RIGHT_FLYWHEEL_ID, MotorType.kBrushless);
  private final CANSparkFlex feederMotor =
      new CANSparkFlex(Constants.RobotMap.FEEDER_CAN_ID, MotorType.kBrushless);
  private final SparkLimitSwitch limitSwitch =
      feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  public ShooterIOVortexVortexLS() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    leftMotor.getEncoder().setMeasurementPeriod(10);
    rightMotor.getEncoder().setMeasurementPeriod(10);
    leftMotor.getEncoder().setAverageDepth(2);
    rightMotor.getEncoder().setAverageDepth(2);

    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    feederMotor.restoreFactoryDefaults();

    feederMotor.setIdleMode(IdleMode.kBrake);

    feederMotor.setSmartCurrentLimit(30);
    feederMotor.enableVoltageCompensation(12.0);

    feederMotor.getEncoder().setMeasurementPeriod(10);
    feederMotor.getEncoder().setAverageDepth(2);

    feederMotor.setInverted(true);

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 60);
            put(PeriodicFrame.kStatus1, 40);
            put(PeriodicFrame.kStatus2, 40);
            put(PeriodicFrame.kStatus3, 20);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 20);
            put(PeriodicFrame.kStatus6, 20);
          }
        },
        leftMotor,
        rightMotor,
        feederMotor);

    ShooterConstants.SHOOTER_GAINS.applyTo(leftMotor.getPIDController());
    ShooterConstants.SHOOTER_GAINS.applyTo(rightMotor.getPIDController());
  }

  @Override
  public void updateInputs(
      ShooterInputsAutoLogged inputs,
      Shooter.ShooterState shooterState,
      Shooter.FeederState feederState) {
    inputs.leftOutputVoltage = RobotController.getBatteryVoltage() * leftMotor.getAppliedOutput();
    inputs.rightOutputVoltage = RobotController.getBatteryVoltage() * rightMotor.getAppliedOutput();

    inputs.leftDrawAmp = leftMotor.getOutputCurrent();
    inputs.rightDrawAmp = rightMotor.getOutputCurrent();

    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();

    inputs.leftPosDeg = Units.rotationsToDegrees(leftMotor.getEncoder().getPosition());
    inputs.rightPosDeg = Units.rotationsToDegrees(rightMotor.getEncoder().getPosition());

    inputs.leftSpeedRPM = leftMotor.getEncoder().getVelocity();
    inputs.rightSpeedRPM = rightMotor.getEncoder().getVelocity();

    inputs.feederOutputVolts = RobotController.getBatteryVoltage() * feederMotor.getAppliedOutput();
    inputs.feederStatorCurrentAmps = -1;
    inputs.feederSupplyCurrentAmps = feederMotor.getOutputCurrent();
    inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();

    inputs.LSTripped = limitSwitch.isPressed();
  }

  @Override
  public void setMotorSetPoint(double leftRPM, double rightRPM) {
    Logger.recordOutput("Flywheel/Left Setpoint", leftRPM);
    Logger.recordOutput("Flywheel/Right Setpoint", rightRPM);
    leftMotor.getPIDController().setReference(leftRPM, ControlType.kVelocity, 0);
    rightMotor.getPIDController().setReference(rightRPM, ControlType.kVelocity, 0);
  }

  @Override
  public void setFeederVolts(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void setShooterVolts(double lVolts, double rVolts) {
    leftMotor.setVoltage(lVolts);
    rightMotor.setVoltage(rVolts);
  }

  @Override
  public void setDisableOnLimitSwitch(boolean val) {
    limitSwitch.enableLimitSwitch(val);
  }
}
