package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

public class LoggableMotor {
  private String name;
  private DCMotor motor;

  public LoggableMotor(String name, DCMotor motor) {
    this.name = name;
    this.motor = motor;
  }

  public void log(double currentDrawAmps, double volts) {
    String name = String.format("Motors/%s", this.name);

    double torque = motor.getTorque(currentDrawAmps);
    double velocity = motor.getSpeed(torque, volts);

    Logger.getInstance().recordOutput(String.format("%s/CurrentAmps", name), currentDrawAmps);
    Logger.getInstance().recordOutput(String.format("%s/VelocityRPM", name), velocity);
    Logger.getInstance().recordOutput(String.format("%s/Volts", name), volts);
    Logger.getInstance().recordOutput(String.format("%s/TorqueNM", name), torque);
    Logger.getInstance().recordOutput(String.format("%s/PowerWatts", name), torque * velocity);
  }
}
