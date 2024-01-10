package frc.robot.subsystems.swerveIO.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.RedHawkUtil;

public class SwerveModuleIOKrakenNeo implements SwerveModuleIO {
  TalonFX drive;

  public SwerveModuleIOKrakenNeo(ModuleInfo info) {
    drive = new TalonFX(info.getDriveCANId());

    TalonFXConfiguration config = new TalonFXConfiguration();
    info.getDriveGains().applyTo(config);
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    RedHawkUtil.applyConfigs(drive, config);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveCurrentDrawAmps = drive.getStatorCurrent().getValue();
    inputs.driveEncoderPositionMetres = drive.getPosition().getValue();
    inputs.driveEncoderVelocityMetresPerSecond = drive.getVelocity().getValue();
    inputs.driveOutputVolts = drive.getMotorVoltage().getValue();
    inputs.driveTempCelcius = drive.getDeviceTemp().getValue();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setAzimuthVoltage'");
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    drive.setControl(new VoltageOut(driveVolts));
  }

  @Override
  public void seed() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'seed'");
  }

  @Override
  public void setAzimuthPositionSetpoint(double setpointDegrees) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setAzimuthPositionSetpoint'");
  }

  @Override
  public void setDriveVelocitySetpoint(double setpointMetersPerSecond, double staticFFVolts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVelocitySetpoint'");
  }
}
