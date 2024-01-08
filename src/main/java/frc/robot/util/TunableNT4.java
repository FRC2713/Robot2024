package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.EnumSet;
import java.util.function.Consumer;

public class TunableNT4 {
  private DoubleSubscriber ntSub;
  private double cachedValue;

  public TunableNT4(String key, double defaultValue) {
    DoubleTopic topic = NetworkTableInstance.getDefault().getTable("Tunables").getDoubleTopic(key);
    topic.publish().set(defaultValue);
    ntSub = topic.subscribe(defaultValue);
    cachedValue = defaultValue;
  }

  public TunableNT4(String key, double defaultValue, Consumer<Double> consumer) {
    this(key, defaultValue);
    addHook(consumer);
  }

  public double getCachedValue() {
    return cachedValue;
  }

  public void addHook(Consumer<Double> consumer) {
    if (!DriverStation.isFMSAttached()) {
      NetworkTableInstance.getDefault()
          .addListener(
              ntSub,
              EnumSet.of(NetworkTableEvent.Kind.kValueAll),
              event -> {
                consumer.accept(event.valueData.value.getDouble());
                cachedValue = event.valueData.value.getDouble();
              });
    }
  }
}
