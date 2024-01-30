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
  @Getter
  private double kP, kI, kD, kS, kV, kG; // TODO: apply kg (or make seperate ElevatorGains class)
  @Getter private String name;
  @Getter private TunableNT4 tunableKP, tunableKI, tunableKD, tunableKS, tunableKV, tunableKG;

  public PIDFFGains buildTunables() {
    // tunableKP =
    //     new TunableNT4(
    //         name + "/kP",
    //         kP,
    //         x -> {
    //           this.kP = x;
    //         });
    // tunableKI =
    //     new TunableNT4(
    //         name + "/kI",
    //         kI,
    //         x -> {
    //           this.kI = x;
    //         });
    // tunableKD =
    //     new TunableNT4(
    //         name + "/kD",
    //         kD,
    //         x -> {
    //           this.kD = x;
    //         });
    // tunableKS =
    //     new TunableNT4(
    //         name + "/kS",
    //         kS,
    //         x -> {
    //           this.kS = x;
    //         });
    // tunableKV =
    //     new TunableNT4(
    //         name + "/kV",
    //         kV,
    //         x -> {
    //           this.kV = x;
    //         });

    tunableKG =
        new TunableNT4(
            name + "/kG",
            kG,
            x -> {
              this.kG = x;
            });

    return this;
  }

  public PIDController createWpilibController() {
    PIDController controller = new PIDController(kP, kI, kD);

    if (tunableKP != null) {
      tunableKP.addHook(x -> controller.setP(x));
      tunableKI.addHook(x -> controller.setI(x));
      tunableKD.addHook(x -> controller.setD(x));
    }

    return controller;
  }

  // public

  public RHRFeedForward createRHRFeedForward() {
    RHRFeedForward ff = RHRFeedForward.builder().kS(kS).kV(kV).build();

    if (tunableKS != null) {
      tunableKS.addHook(x -> ff.setKS(x));
      tunableKV.addHook(x -> ff.setKV(x));
    }

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

    if (tunableKP != null) {
      tunableKP.addHook(x -> controller.setP(x));
      tunableKI.addHook(x -> controller.setI(x));
      tunableKD.addHook(x -> controller.setD(x));
      tunableKS.addHook(x -> controller.getFeedForward().setKS(x));
      tunableKV.addHook(x -> controller.getFeedForward().setKV(x));
    }
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

    if (tunableKS != null) {
      tunableKP.addHook(x -> controller.setP(x));
      tunableKI.addHook(x -> controller.setI(x));
      tunableKD.addHook(x -> controller.setD(x));
      tunableKV.addHook(x -> controller.setFF(x));
    }
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
