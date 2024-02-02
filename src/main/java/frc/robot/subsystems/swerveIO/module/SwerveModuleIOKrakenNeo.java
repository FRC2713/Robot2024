package frc.robot.subsystems.swerveIO.module;

import static frc.robot.util.RedHawkUtil.cOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    config.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    RedHawkUtil.applyConfigs(drive, config);

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(info.getAziEncoderCANId(), info.getOffset());
    azimuth = new CANSparkMax(info.getAziCANId(), MotorType.kBrushless);
    azimuth.restoreFactoryDefaults();
    azimuth.setCANTimeout(Constants.CAN_TIMEOUT_MS);

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

    azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT);

    for (int i = 0; i < 30; i++) {
      azimuth.setInverted(true);
    }

    cOk(azimuth.setIdleMode(IdleMode.kBrake));

    cOk(getAziEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0));
    cOk(getAziEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0));

    SparkConfigurator azimuthConfigurator = new SparkConfigurator(azimuth);
    azimuthConfigurator
        .setInverted(true)
        .checkOK(s -> s.setIdleMode(IdleMode.kBrake))
        .checkOKAndReadBackValue(
            s -> s.setIdleMode(IdleMode.kBrake), s -> s.getIdleMode() == IdleMode.kBrake)
        .checkOK(s -> s.getEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0))
        .checkOK(s -> s.getEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0));

    info.getAzimuthGains().applyTo(azimuth.getPIDController());
    azimuth.getPIDController().setPositionPIDWrappingEnabled(true);
    azimuth.getPIDController().setPositionPIDWrappingMinInput(-180);
    azimuth.getPIDController().setPositionPIDWrappingMaxInput(180);

    for (int i = 0; i < 30; i++) {
      // seed();
    }

    azimuth.setCANTimeout(0);

    azimuth.burnFlash();
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveCurrentDrawAmps = drive.getStatorCurrent().getValue();
    inputs.driveEncoderPositionMetres =
        drive.getPosition().getValue() * Constants.DriveConstants.DIST_PER_PULSE;
    inputs.driveEncoderVelocityMetresPerSecond =
        drive.getVelocity().getValue() * Constants.DriveConstants.DIST_PER_PULSE;
    inputs.driveOutputVolts = drive.getMotorVoltage().getValue();
    inputs.driveTempCelcius = drive.getDeviceTemp().getValue();
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
        setpointMetersPerSecond / Constants.DriveConstants.DIST_PER_PULSE;
    final VelocityVoltage m_request = new VelocityVoltage(desiredRotationsPerSecond).withSlot(0);
    double ffVolts = ff.calculate(setpointMetersPerSecond);
    Logger.recordOutput(
        "Swerve/" + info.getName() + "/Drive Setpoint MPS", setpointMetersPerSecond);
    Logger.recordOutput(
        "Swerve/" + info.getName() + "/Drive Setpoint RPS", desiredRotationsPerSecond);
    Logger.recordOutput("Swerve/" + info.getName() + "/Drive kV and kS", ffVolts);
    drive.setControl(m_request.withFeedForward(ffVolts));
  }
}
