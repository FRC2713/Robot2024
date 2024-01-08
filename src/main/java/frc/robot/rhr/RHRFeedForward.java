package frc.robot.rhr;

import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
public class RHRFeedForward {
  @Getter @Setter private double kS, kV;

  public double calculate(double velocity) {
    return this.calculate(velocity, 0);
  }

  public double calculate(double velocity, double acceleration) {
    return kS * Math.signum(velocity) + kV * velocity;
  }
}
