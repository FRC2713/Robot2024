package frc.robot.subsystems.shooterIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.RedHawkUtil;

public class ShooterIOVortex implements ShooterIO {
  private final CANSparkFlex leftMotor =
      new CANSparkFlex(
          Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID,
          MotorType.kBrushless); // left and right should swap
  private final CANSparkFlex rightMotor =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_RIGHT_FLYWHEEL_ID, MotorType.kBrushless);
  private final TalonFX feeder = new TalonFX(Constants.RobotMap.FEEDER_CAN_ID);
  private final SparkAnalogSensor sensor;

  private StatusSignal<Double> feederMotorVoltage = feeder.getMotorVoltage();
  private StatusSignal<Double> feederSupplyCurrent = feeder.getSupplyCurrent();
  private StatusSignal<Double> feederStatorCurrent = feeder.getStatorCurrent();
  private StatusSignal<Double> feederVelocity = feeder.getVelocity();

  public ShooterIOVortex() {
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

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;
    RedHawkUtil.applyConfigs(feeder, config);

    ShooterConstants.SHOOTER_GAINS.applyTo(leftMotor.getPIDController());
    ShooterConstants.SHOOTER_GAINS.applyTo(rightMotor.getPIDController());

    sensor = rightMotor.getAnalog(Mode.kAbsolute);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, feederMotorVoltage, feederSupplyCurrent, feederStatorCurrent, feederVelocity);
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs, Shooter.State state) {
    BaseStatusSignal.refreshAll(
        feederMotorVoltage, feederSupplyCurrent, feederStatorCurrent, feederVelocity);

    inputs.leftOutputVoltage = leftMotor.getBusVoltage();
    inputs.rightOutputVoltage = rightMotor.getBusVoltage();

    inputs.leftDrawAmp = leftMotor.getOutputCurrent();
    inputs.rightDrawAmp = rightMotor.getOutputCurrent();

    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();

    inputs.leftPosDeg = Units.rotationsToDegrees(leftMotor.getEncoder().getPosition());
    inputs.rightPosDeg = Units.rotationsToDegrees(rightMotor.getEncoder().getPosition());

    inputs.leftSpeedRPM = leftMotor.getEncoder().getVelocity();
    inputs.rightSpeedRPM = rightMotor.getEncoder().getVelocity();

    inputs.feederOutputVolts = feederMotorVoltage.getValue();
    inputs.feederStatorCurrentAmps = feederStatorCurrent.getValue();
    inputs.feederSupplyCurrentAmps = feederSupplyCurrent.getValue();
    inputs.feederVelocityRPM = feederVelocity.getValue();

    inputs.sensorVoltage = sensor.getVoltage();
  }

  @Override
  public void setMotorSetPoint(double leftRPM, double rightRPM) {
    leftMotor.getPIDController().setReference(leftRPM, ControlType.kVelocity, 0);
    rightMotor.getPIDController().setReference(rightRPM, ControlType.kVelocity, 0);
  }

  @Override
  public void setFeederVolts(double volts) {
    feeder.setControl(new VoltageOut(volts));
  }
}
