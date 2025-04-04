package frc.lib.util;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldLayout;

public class FlippingUtil {
    
  /**
   * Flip a field position to the other side of the field, maintaining a blue alliance origin
   */
  public static Translation2d flipFieldPosition(Translation2d pos) {
    return new Translation2d(FieldLayout.FIELD_LENGTH - pos.getX(), FieldLayout.FIELD_WIDTH - pos.getY());
  }

  /**
   * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
   */
  public static Rotation2d flipFieldRotation(Rotation2d rotation) {
    return rotation.minus(Rotation2d.kPi);
  }

  /**
   * Flip a field pose to the other side of the field, maintaining a blue alliance origin
   */
  public static Pose2d flipFieldPose(Pose2d pose) {
    return new Pose2d(
        flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
  }

  public static Pose2d flipJustY(Pose2d pose) {
    return new Pose2d(
      pose.getX(), FieldLayout.FIELD_WIDTH - pose.getY(), flipFieldRotation(pose.getRotation())
    );
  }

  public static Pose2d flipJustX(Pose2d pose) {
    return new Pose2d(
      FieldLayout.FIELD_LENGTH - pose.getX(), pose.getY(), flipFieldRotation(pose.getRotation())
    );
  }
}
