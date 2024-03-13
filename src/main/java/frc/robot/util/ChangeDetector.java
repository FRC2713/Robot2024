package frc.robot.util;

import java.util.function.Consumer;

public class ChangeDetector<T> {
  private Consumer<T> consumer;
  private T value;

  public ChangeDetector(Consumer<T> consumer) {
    this.consumer = consumer;
  }

  public void feed(T newValue) {
    if (value == null) {
      if (newValue != null) {
        consumer.accept(newValue);
      }
    } else {
      if (!value.equals(newValue)) {
        consumer.accept(newValue);
      }
    }

    this.value = newValue;
  }
}
