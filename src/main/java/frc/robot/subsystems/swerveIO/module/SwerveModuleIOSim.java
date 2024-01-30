package frc.robot.subsystems.swerveIO.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.rhr.RHRPIDFFController;
import frc.robot.util.PIDFFGains;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.0004096955);
  FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.12, 0.0025);

  RHRPIDFFController driveController, azimuthController;

  double theAziVolts = 0;
  double theDriveVolts = 0;
  double driveFFVolts = 0;

  ModuleInfo info;

  public SwerveModuleIOSim(ModuleInfo information) {
    driveController =
        PIDFFGains.builder()
            .name("Swerve/Sim/Driving")
            .kP(1.2)
            .kV(2.3)
            .build()
            .createRHRController();
    azimuthController =
        PIDFFGains.builder()
            .name("Swerve/Sim/Azimuth")
            .kP(0.1)
            // .kS(0.225)
            .build()
            .createRHRController();
    azimuthController.enableContinuousInput(-180, 180);
    info = information;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    setAzimuthVoltage(
        MathUtil.clamp(azimuthController.calculate(inputs.aziEncoderPositionDeg), -12, 12));
    setDriveVoltage(
        MathUtil.clamp(
            driveController.calculate(inputs.driveEncoderVelocityMetresPerSecond) + driveFFVolts,
            -12,
            12));

    azimuthSim.update(0.02);
    driveSim.update(0.02);

    inputs.aziAbsoluteEncoderRawVolts = 0;
    inputs.aziAbsoluteEncoderAdjVolts = 0;
    inputs.aziAbsoluteEncoderAdjAngleDeg = 0;
    inputs.aziOutputVolts = MathUtil.clamp(theAziVolts, -12.0, 12.0);
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
    inputs.driveOutputVolts = MathUtil.clamp(theDriveVolts, -12.0, 12.0);
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

  @Override
  public void setAzimuthPositionSetpoint(double setpointDegrees) {
    azimuthController.setSetpoint(setpointDegrees);
  }

  @Override
  public void setDriveVelocitySetpoint(double setpointMetersPerSecond, double staticFFVolts) {
    driveController.setSetpoint(setpointMetersPerSecond);
    // driveFFVolts = staticFFVolts;
  }
}
