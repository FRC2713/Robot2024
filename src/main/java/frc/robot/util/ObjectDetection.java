package frc.robot.util;

import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import lombok.Getter;
import lombok.NonNull;
import lombok.Setter;
import org.opencv.core.Point;

public class ObjectDetection {
  @Getter @Setter public Point centre;

  @Getter @Setter public double width;

  @Getter @Setter public double height;

  public ObjectDetection(@NonNull Point centrexy, double width, double height) {
    centre = centrexy;
    this.width = width;
    this.height = height;
  }

  public double area() {
    return width * height;
  }

  public double goodness() {
    return area();
  }

  public static ObjectDetection fromImageRelative(
      double imwidth, double imheight, double tlx, double tly, double width, double height) {
    tlx /= imwidth;
    tly /= imheight;
    width /= imwidth;
    height /= imheight;

    return new ObjectDetection(new Point(tlx + (width / 2), tly + (height / 2)), width, height);
  }

  public static ObjectDetection fromImageRelativeCentre(
      double imwidth, double imheight, double tlx, double tly, double width, double height) {
    tlx /= imwidth;
    tly /= imheight;
    width /= imwidth;
    height /= imheight;

    return new ObjectDetection(new Point(tlx, tly), width, height);
  }

  public static ObjectDetection fromLimelight(LimelightTarget_Detector detection) {
    return new ObjectDetection(
        new Point(detection.tx, detection.ty), Math.sqrt(detection.ta), Math.sqrt(detection.ta));
  }
}
