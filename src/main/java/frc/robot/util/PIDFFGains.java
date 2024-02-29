package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.rhr.RHRFeedForward;
import frc.robot.rhr.RHRPIDFFController;
import lombok.Builder;
import lombok.Getter;

@Builder
public class PIDFFGains {
  @Getter private double kP, kI, kD, kS, kV, kG;
  @Getter private String name;

  public PIDFFGains buildTunables() {
    return this;
  }

  public PIDController createWpilibController() {
    PIDController controller = new PIDController(kP, kI, kD);

    return controller;
  }

  // public

  public RHRFeedForward createRHRFeedForward() {
    RHRFeedForward ff = RHRFeedForward.builder().kS(kS).kV(kV).build();

    return ff;
  }

  public ElevatorFeedforward createElevatorFeedforward() {
    return new ElevatorFeedforward(kS, kG, kV);
  }

  public ProfiledPIDController createProfiledPIDController(
      TrapezoidProfile.Constraints constraints) {
    return new ProfiledPIDController(kP, kI, kD, constraints);
  }

  public ArmFeedforward createArmFeedForward() {
    return new ArmFeedforward(kS, kG, kV);
  }

  public RHRPIDFFController createRHRController() {
    RHRPIDFFController controller = new RHRPIDFFController(this);

    return controller;
  }

  public void applyTo(TalonFXConfiguration config) {
    config.Slot0.kP = this.kP;
    config.Slot0.kI = this.kI;
    config.Slot0.kD = this.kD;
    config.Slot0.kS = this.kS;
    config.Slot0.kV = this.kV;
  }

  public void applyTo(SparkPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kV);
  }

  public void applyTo(SparkPIDController controller, int slotId) {
    controller.setP(kP, slotId);
    controller.setI(kI, slotId);
    controller.setD(kD, slotId);
    controller.setFF(kV, slotId);
  }

  public String toString() {
    return String.format("kP = %s / kD = %s", kP, kD);
  }

  public PIDConstants toPathplannerGains() {
    return new PIDConstants(kP, kI, kD);
  }

  public static PIDFFGains fromPIDGains(PIDController controller) {
    return PIDFFGains.builder()
        .kP(controller.getP())
        .kI(controller.getI())
        .kD(controller.getD())
        .build();
  }
}
