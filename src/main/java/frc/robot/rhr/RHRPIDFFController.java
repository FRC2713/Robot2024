package frc.robot.rhr;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.PIDFFGains;

public class RHRPIDFFController extends PIDController {
  private RHRFeedForward feedforward;

  public RHRPIDFFController(PIDFFGains gains) {
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

  public RHRFeedForward getFeedForward() {
    return feedforward;
  }
}
