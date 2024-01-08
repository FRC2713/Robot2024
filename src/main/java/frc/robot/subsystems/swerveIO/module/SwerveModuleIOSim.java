package frc.robot.subsystems.swerveIO.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);
  FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.12, 0.025);

  double theAziVolts = 0;
  double theDriveVolts = 0;

  private final ModuleInfo information;

  public SwerveModuleIOSim(ModuleInfo information) {
    this.information = information;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    azimuthSim.update(0.02);
    driveSim.update(0.02);

    inputs.aziAbsoluteEncoderRawVolts = 0;
    inputs.aziAbsoluteEncoderAdjVolts = 0;
    inputs.aziAbsoluteEncoderAdjAngleDeg = 0;
    inputs.aziOutputVolts = MathUtil.clamp(azimuthSim.getOutput(0), -12.0, 12.0);
    inputs.aziTempCelcius = 0.0;
    inputs.aziCurrentDrawAmps = azimuthSim.getCurrentDrawAmps();
    inputs.aziEncoderPositionDeg +=
        Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec()) * 0.02;
    inputs.aziEncoderVelocityDegPerSecond =
        Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec());

    inputs.driveEncoderPositionMetres +=
        driveSim.getAngularVelocityRPM() * Math.PI * Units.inchesToMeters(4) / 60 * 0.02;

    inputs.driveEncoderVelocityMetresPerSecond =
        driveSim.getAngularVelocityRPM() * Math.PI * Units.inchesToMeters(4) / 60;
    inputs.driveOutputVolts = MathUtil.clamp(driveSim.getOutput(0), -12.0, 12.0);
    inputs.driveCurrentDrawAmps = driveSim.getCurrentDrawAmps();
    inputs.driveTempCelcius = 0.0;
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    theAziVolts = aziVolts;
    azimuthSim.setInputVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    theDriveVolts = driveVolts;
    driveSim.setInputVoltage(driveVolts);
  }

  public void seed() {}
}
