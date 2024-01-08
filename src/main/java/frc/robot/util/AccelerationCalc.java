package frc.robot.util;

import edu.wpi.first.math.Pair;

public class AccelerationCalc {
  private RollingBuffer<Pair<Double, Double>> velocityAndTimes;

  public AccelerationCalc(int n) {
    velocityAndTimes = new RollingBuffer<>(n);
  }

  public void addVelocity(double velocity, double timestamp) {
    velocityAndTimes.addData(new Pair<Double, Double>(velocity, timestamp));
  }

  public double getAcceleration() {
    Pair<Double, Double> mostRecent = velocityAndTimes.getLatest();
    Pair<Double, Double> oldest = velocityAndTimes.getOldest();

    if (mostRecent == null || oldest == null) {
      return 0;
    }

    return (mostRecent.getFirst() - oldest.getFirst())
        / (mostRecent.getSecond() - oldest.getSecond());
  }

  public double calculate(double velocity, double timestamp) {
    addVelocity(velocity, timestamp);
    return getAcceleration();
  }
}
