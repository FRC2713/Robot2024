package frc.robot.subsystems.swerveIO.module;

import static frc.robot.util.RedHawkUtil.cOk;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SparkConfigurator;
import java.util.HashMap;

public class SwerveModuleIOSparkMAX implements SwerveModuleIO {

  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  CANSparkMax driver;
  CANSparkMax azimuth;
  private final ModuleInfo information;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziAbsoluteEncoder() {
    return azimuthEncoder;
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public SwerveModuleIOSparkMAX(ModuleInfo information) {
    this.information = information;
    azimuthEncoder =
        new OffsetAbsoluteAnalogEncoder(
            this.information.getAziEncoderCANId(), this.information.getOffset());
    driver = new CANSparkMax(this.information.getDriveCANId(), MotorType.kBrushless);
    azimuth = new CANSparkMax(this.information.getAziCANId(), MotorType.kBrushless);

    driver.restoreFactoryDefaults();
    azimuth.restoreFactoryDefaults();

    driver.setCANTimeout(Constants.CAN_TIMEOUT_MS);
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
        driver,
        azimuth);

    driver.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
    azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT);

    for (int i = 0; i < 30; i++) {
      azimuth.setInverted(true);
      driver.setInverted(true);
    }

    cOk(driver.setIdleMode(IdleMode.kBrake));
    cOk(azimuth.setIdleMode(IdleMode.kBrake));

    cOk(
        getDriveEncoder()
            .setPositionConversionFactor(
                RedHawkUtil.getDistPerPulse(information.getWheelDiameter())));
    cOk(
        getDriveEncoder()
            .setVelocityConversionFactor(
                RedHawkUtil.getDistPerPulse(information.getWheelDiameter())));

    cOk(getAziEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0));
    cOk(getAziEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0));

    new SparkConfigurator<>(azimuth)
        .setUntilOk(() -> azimuth.setIdleMode(IdleMode.kBrake))
        .setUntilOk(() -> azimuth.getEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0))
        .setUntilOk(() -> azimuth.getEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0));

    information.getDriveGains().applyTo(driver.getPIDController());
    information.getAzimuthGains().applyTo(azimuth.getPIDController());
    azimuth.getPIDController().setPositionPIDWrappingEnabled(true);
    azimuth.getPIDController().setPositionPIDWrappingMinInput(-180);
    azimuth.getPIDController().setPositionPIDWrappingMaxInput(180);

    for (int i = 0; i < 30; i++) {
      // seed();
    }

    driver.setCANTimeout(0);
    azimuth.setCANTimeout(0);

    driver.burnFlash();
    azimuth.burnFlash();
  }

  public void seed() {
    cOk(getAziEncoder().setPosition(getAziAbsoluteEncoder().getAdjustedRotation2d().getDegrees()));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // inputs.aziAbsoluteEncoderRawVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjAngleDegReal = getAziEncoder().getPosition();

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

    inputs.driveEncoderPositionMetres = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocityMetresPerSecond = getDriveEncoder().getVelocity();
    inputs.driveOutputVolts = driver.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentDrawAmps = driver.getOutputCurrent();
    inputs.driveTempCelcius = driver.getMotorTemperature();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }

  @Override
  public void setAzimuthPositionSetpoint(double setpointDegrees) {
    azimuth.getPIDController().setReference(setpointDegrees, ControlType.kPosition);
  }

  @Override
  public void setDriveVelocitySetpoint(double setpointMetersPerSecond, double staticFFVolts) {
    driver
        .getPIDController()
        .setReference(
            setpointMetersPerSecond,
            ControlType.kVelocity,
            0,
            staticFFVolts * Math.signum(setpointMetersPerSecond),
            ArbFFUnits.kVoltage);
  }
}
