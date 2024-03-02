package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class SparkConfigurator<T extends CANSparkBase> {
  private T spark;
  private int n;

  public SparkConfigurator(T spark) {
    this.spark = spark;
  }

  public SparkConfigurator<T> setUntilOk(Supplier<REVLibError> mutator) {
    n++;
    REVLibError err;
    do {
      err = mutator.get();
      if (err != REVLibError.kOk) {
        System.err.println(
            String.format("Spark #%s (n=%s) error: %s", spark.getDeviceId(), n, err.name()));
        Timer.delay(Units.millisecondsToSeconds(5));
      }
    } while (err != REVLibError.kOk);

    return this;
  }
}
