package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class ErrorTracker {
  List<Pose2d> tracking_error_over_time_;
  int max_num_samples_;
  PIDFFGains translationGains, rotationGains;

  public ErrorTracker(int max_num_samples, PIDFFGains translationGains, PIDFFGains rotationGains) {
    max_num_samples_ = max_num_samples;
    tracking_error_over_time_ = new ArrayList<Pose2d>(max_num_samples);

    this.translationGains = translationGains;
    this.rotationGains = rotationGains;
  }

  public void addObservation(Pose2d error) {
    if (tracking_error_over_time_.size() > max_num_samples_) {
      tracking_error_over_time_.remove(0);
    }
    tracking_error_over_time_.add(error);
  }

  public void reset() {
    tracking_error_over_time_.clear();
  }

  public Translation2d getMaxTranslationError() {
    if (tracking_error_over_time_.isEmpty()) {
      return new Translation2d();
    }
    double max_norm = Double.NEGATIVE_INFINITY;
    Translation2d max = null;
    for (var error : tracking_error_over_time_) {
      double norm2 = error.getTranslation().getNorm() * error.getTranslation().getNorm();
      if (norm2 > max_norm) {
        max_norm = norm2;
        max = error.getTranslation();
      }
    }
    return max;
  }

  public Rotation2d getMaxRotationError() {
    if (tracking_error_over_time_.isEmpty()) return new Rotation2d();
    double max_norm = Double.NEGATIVE_INFINITY;
    Rotation2d max = null;
    for (var error : tracking_error_over_time_) {
      double norm = Math.abs(error.getRotation().getRadians());
      if (norm > max_norm) {
        max_norm = norm;
        max = error.getRotation();
      }
    }
    return max;
  }

  public double getTranslationRMSE() {
    double error_sum = 0.0;
    for (var error : tracking_error_over_time_) {
      error_sum += (error.getTranslation().getNorm() * error.getTranslation().getNorm());
    }
    error_sum /= (double) tracking_error_over_time_.size();
    return Math.sqrt(error_sum);
  }

  public double getRotationRMSE() {
    double error_sum = 0.0;
    for (var error : tracking_error_over_time_) {
      error_sum += error.getRotation().getRadians() * error.getRotation().getRadians();
    }
    error_sum /= (double) tracking_error_over_time_.size();
    return Math.sqrt(error_sum);
  }

  public void printSummary(String name) {
    if (tracking_error_over_time_.isEmpty()) {
      System.out.println("<empty tracker>");
      return;
    }

    System.out.println("[" + name + "] Error Summary---");
    System.out.println(
        "Translation: RMSE " + getTranslationRMSE() + ", Max: " + getMaxTranslationError());
    System.out.println("Rotation: RMSE " + getRotationRMSE() + ", Max: " + getMaxRotationError());

    System.out.println("Rotation RMSE;Rotation MAX;Translation RMSE;Translation MAX");
    System.out.println(
        String.format(
            "%f,%f,%f,%s,%f,%s,%f",
            translationGains.getKP(),
            translationGains.getKD(),
            getRotationRMSE(),
            getMaxRotationError().getDegrees(),
            getTranslationRMSE(),
            getMaxTranslationError().getX(),
            getMaxTranslationError().getY()));
  }
}
