package frc.robot.subsystems.swerveIO.module;

import static frc.robot.util.RedHawkUtil.cOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.rhr.RHRFeedForward;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SparkConfigurator;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOKrakenNeo implements SwerveModuleIO {
  TalonFX drive;
  CANSparkMax azimuth;
  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  private ModuleInfo info;
  private RHRFeedForward ff;

  private SparkConfigurator<CANSparkMax> configurator;

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziAbsoluteEncoder() {
    return azimuthEncoder;
  }

  public SwerveModuleIOKrakenNeo(ModuleInfo info) {
    this.info = info;
    ff = info.getDriveGains().createRHRFeedForward();
    drive = new TalonFX(info.getDriveCANId());

    TalonFXConfiguration config = new TalonFXConfiguration();
    info.getDriveGains().applyTo(config);
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = 40;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;
    RedHawkUtil.applyConfigs(drive, config);

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(info.getAziEncoderCANId(), info.getOffset());
    azimuth = new CANSparkMax(info.getAziCANId(), MotorType.kBrushless);
    azimuth.restoreFactoryDefaults();

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 60);
            put(PeriodicFrame.kStatus1, 20);
            put(PeriodicFrame.kStatus2, 20);
            put(PeriodicFrame.kStatus3, 65535);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 65535);
            put(PeriodicFrame.kStatus6, 65535);
          }
        },
        azimuth);

    configurator = new SparkConfigurator<>(azimuth);
    configurator
        .setUntilOk(() -> azimuth.setIdleMode(IdleMode.kBrake))
        .setUntilOk(() -> azimuth.getEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0))
        .setUntilOk(() -> azimuth.getEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0))
        .setUntilOk(() -> azimuth.getPIDController().setPositionPIDWrappingEnabled(true))
        .setUntilOk(() -> CANSparkMaxExtensions.setInverted(azimuth, true))
        .setUntilOk(() -> azimuth.setIdleMode(IdleMode.kBrake))
        .setUntilOk(() -> azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT))
        .setUntilOk(() -> azimuth.getPIDController().setPositionPIDWrappingEnabled(true))
        .setUntilOk(() -> azimuth.getPIDController().setPositionPIDWrappingMinInput(-180))
        .setUntilOk(() -> azimuth.getPIDController().setPositionPIDWrappingMaxInput(180))
        .setUntilOk(() -> azimuth.getPIDController().setP(info.getAzimuthGains().getKP()));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveCurrentDrawAmps = drive.getStatorCurrent().getValue();
    inputs.driveEncoderPositionMetres =
        drive.getPosition().getValue() * RedHawkUtil.getDistPerPulse(info.getWheelDiameter());
    Logger.recordOutput(
        "Swerve/" + info.getName() + "/Raw Drive Angle", drive.getPosition().getValue());
    inputs.driveEncoderVelocityMetresPerSecond =
        drive.getVelocity().getValue() * RedHawkUtil.getDistPerPulse(info.getWheelDiameter());
    inputs.driveOutputVolts = drive.getMotorVoltage().getValue();
    inputs.driveTempCelcius = drive.getDeviceTemp().getValue();

    inputs.aziAbsoluteEncoderRawVolts = azimuthEncoder.getUnadjustedVoltage();
    inputs.aziAbsoluteEncoderAdjVolts = azimuthEncoder.getAdjustedVoltage();
    inputs.aziAbsoluteEncoderAdjAngleDeg = azimuthEncoder.getAdjustedRotation2d().getDegrees();

    inputs.aziOutputVolts = azimuth.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.aziTempCelcius = azimuth.getMotorTemperature();
    inputs.aziCurrentDrawAmps = azimuth.getOutputCurrent();
    inputs.aziEncoderPositionDeg = getAziEncoder().getPosition();
    inputs.aziEncoderVelocityDegPerSecond = getAziEncoder().getVelocity();
    inputs.aziEncoderSimplifiedPositionDeg =
        OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                Rotation2d.fromDegrees(getAziEncoder().getPosition()))
            .getDegrees();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    drive.setControl(new VoltageOut(driveVolts));
  }

  @Override
  public void seed() {
    cOk(getAziEncoder().setPosition(getAziAbsoluteEncoder().getAdjustedRotation2d().getDegrees()));
  }

  @Override
  public void setAzimuthPositionSetpoint(double setpointDegrees) {
    azimuth.getPIDController().setReference(setpointDegrees, ControlType.kPosition);
  }

  @Override
  public void setDriveVelocitySetpoint(double setpointMetersPerSecond, double staticFFVolts) {
    var desiredRotationsPerSecond =
        setpointMetersPerSecond / RedHawkUtil.getDistPerPulse(info.getWheelDiameter());
    final VelocityTorqueCurrentFOC m_request =
        new VelocityTorqueCurrentFOC(desiredRotationsPerSecond);
    // double ffVolts = ff.calculate(setpointMetersPerSecond);
    Logger.recordOutput(
        "Swerve/" + info.getName() + "/Drive Setpoint MPS", setpointMetersPerSecond);
    Logger.recordOutput(
        "Swerve/" + info.getName() + "/Drive Setpoint RPS", desiredRotationsPerSecond);
    // Logger.recordOutput("Swerve/" + info.getName() + "/Drive kV and kS", ffVolts);
    drive.setControl(m_request);
  }

  @Override
  public void setDriveCurrentLimit(int amps) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = amps;
    RedHawkUtil.applyConfigs(drive, config);
  }
}
