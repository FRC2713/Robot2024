package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.rhr.RHRFeedForward;
import frc.robot.rhr.RHRPIDFFController;
import lombok.Builder;
import lombok.Getter;

@Builder
public class PIDFFGains {
  @Getter private double kP, kI, kD, kS, kV;
  @Getter private String name;
  @Getter private TunableNT4 tunableKP, tunableKI, tunableKD, tunableKS, tunableKV;

  public PIDFFGains buildTunables() {
    tunableKP =
        new TunableNT4(
            name + "/kP",
            kP,
            x -> {
              this.kP = x;
            });
    tunableKI =
        new TunableNT4(
            name + "/kI",
            kI,
            x -> {
              this.kI = x;
            });
    tunableKD =
        new TunableNT4(
            name + "/kD",
            kD,
            x -> {
              this.kD = x;
            });
    tunableKS =
        new TunableNT4(
            name + "/kS",
            kS,
            x -> {
              this.kS = x;
            });
    tunableKV =
        new TunableNT4(
            name + "/kV",
            kV,
            x -> {
              this.kV = x;
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

  public RHRFeedForward createRHRFeedForward() {
    RHRFeedForward ff = RHRFeedForward.builder().kS(kS).kV(kV).build();

    if (tunableKS != null) {
      tunableKS.addHook(x -> ff.setKS(x));
      tunableKV.addHook(x -> ff.setKV(x));
    }

    return ff;
  }

  public RHRPIDFFController createRHRController() {
    return new RHRPIDFFController(this);
  }
}
