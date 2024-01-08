package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

public class RollingBuffer<T> {
  private int maxSize;
  private Deque<T> data;

  public RollingBuffer(int maxSize) {
    if (maxSize <= 0) {
      throw new IllegalArgumentException("Max size must be positive");
    }
    this.maxSize = maxSize;
    this.data = new ArrayDeque<>(maxSize);
  }

  public void addData(T value) {
    if (data.size() == maxSize) {
      data.removeFirst();
    }

    data.offerLast(value);
  }

  public T getOldest() {
    return data.peekFirst();
  }

  public T getLatest() {
    return data.peekLast();
  }
}
