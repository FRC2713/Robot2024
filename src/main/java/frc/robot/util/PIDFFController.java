package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.rhr.RHRFeedForward;

public class PIDFFController extends PIDController {
  private RHRFeedForward feedforward;

  public PIDFFController(PIDFFGains gains) {
    super(gains.getKP(), gains.getKI(), gains.getKD());
    feedforward = gains.createRHRFeedForward();
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  @Override
  public double calculate(double measurement) {
    return super.calculate(measurement) + feedforward.calculate(getSetpoint());
  }
}
