package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import java.util.function.Function;

public class SparkConfigurator {
  private CANSparkMax spark;
  private int uniqueConfigs = 0, totalCalls = 0;

  public SparkConfigurator(CANSparkMax spark) {
    this.spark = spark;
  }

  public SparkConfigurator checkOK(Function<CANSparkMax, REVLibError> setConfigCall) {
    uniqueConfigs++;
    REVLibError maybeError = REVLibError.kOk;
    do {
      maybeError = setConfigCall.apply(spark);
      totalCalls++;

      if (maybeError != REVLibError.kOk) {
        System.err.println(
            String.format(
                "[%s] %s setting #%s not OK! (%s)",
                maybeError.name(), spark.getDeviceId(), uniqueConfigs, totalCalls));
      }
    } while (maybeError != REVLibError.kOk);

    return this;
  }

  public <T> SparkConfigurator checkOKAndReadBackValue(
      Function<CANSparkMax, REVLibError> setConfigCall, Function<CANSparkMax, Boolean> isGood) {

    uniqueConfigs++;
    REVLibError maybeError = REVLibError.kOk;
    do {
      maybeError = setConfigCall.apply(spark);
      totalCalls++;

      if (maybeError != REVLibError.kOk) {
        System.err.println(
            String.format(
                "[%s] %s setting #%s not OK! (%s)",
                maybeError.name(), spark.getDeviceId(), uniqueConfigs, totalCalls));
      }
    } while (maybeError != REVLibError.kOk && isGood.apply(spark));

    return this;
  }

  public SparkConfigurator setInverted(boolean inverted) {
    // alt approach: 3005s CAN stuff
    // https://github.com/FRC3005/Offseason-2023/blob/92553a0ea14fc3feee2a5d9d3d135bfe99358ec3/src/main/java/com/revrobotics/CANSparkMaxExtensions.java#L16

    uniqueConfigs++;
    do {
      spark.setInverted(inverted);
      totalCalls++;
      if (spark.getLastError() != REVLibError.kOk) {
        System.err.println(
            String.format(
                "[%s] %s setting #%s not OK! (%s)",
                spark.getLastError().name(), spark.getDeviceId(), uniqueConfigs, totalCalls));
      }
    } while (spark.getInverted() != inverted);

    return this;
  }
}
