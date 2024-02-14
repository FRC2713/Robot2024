package frc.robot.util;

import com.revrobotics.CANSparkBase;

public class SparkConfigurator<T extends CANSparkBase> {
  private T spark;
  private int n;

  public SparkConfigurator(T spark) {
    this.spark = spark;
  }

  public SparkConfigurator<T> setUntilOk(Runnable runnable) {
    n++;
    // do {
    //   runnable.run();
    //   if (spark.getLastError() != REVLibError.kOk) {
    //     System.err.println(
    //         String.format(
    //             "Spark #%s (n=%s) last error: %s",
    //             spark.getDeviceId(), n, spark.getLastError().name()));
    //   }
    // } while (spark.getLastError() != REVLibError.kOk);

    runnable.run();
    return this;
  }
}
