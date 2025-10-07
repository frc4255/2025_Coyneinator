package frc.robot.sim;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/**
 * Computes articulated component poses for the Coyneinator 3D robot model and logs them for the
 * AdvantageScope Field3d renderer.
 */
public class Coyneinator3dVisualizer {
  private static final Translation3d STAGE1_ZERO_POS =
      new Translation3d(0.129388, 0.266700, 0.304800);
  private static final Translation3d STAGE2_ZERO_POS =
      new Translation3d(0.129388, 0.266700, 0.304800);
  private static final Translation3d STAGE3_ZERO_POS =
      new Translation3d(0.129388, 0.266700, 0.304800);
  private static final Translation3d STAGE4_ZERO_POS =
      new Translation3d(-0.046838, 0.113538, -0.863524);

  // Estimated maximum travel distance of the elevator inner stage (meters).
  private static final double MAX_ELEVATOR_EXTENSION_METERS = 0.78;

  private static final Translation3d ELEVATOR_EXTENSION_AXIS = new Translation3d(0.0, 0.0, 1.0);

  private final Swerve swerve;
  private final Pivot pivot;
  private final Elevator elevator;
  private final WristPitch wristPitch;
  private final WristRoll wristRoll;

  public Coyneinator3dVisualizer(
      Swerve swerve,
      Pivot pivot,
      Elevator elevator,
      WristPitch wristPitch,
      WristRoll wristRoll) {
    this.swerve = Objects.requireNonNull(swerve);
    this.pivot = Objects.requireNonNull(pivot);
    this.elevator = Objects.requireNonNull(elevator);
    this.wristPitch = Objects.requireNonNull(wristPitch);
    this.wristRoll = Objects.requireNonNull(wristRoll);
  }

  /** Updates the logged poses. Intended to be called once per robot loop in simulation. */
  public void update() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    Pose2d basePose2d = swerve.getPose();
    Pose3d basePose =
        new Pose3d(
            basePose2d.getX(),
            basePose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, basePose2d.getRotation().getRadians()));

    double pivotAngle = pivot.getPivotPosition();
    double elevatorExtension =
        Math.max(0.0, Math.min(MAX_ELEVATOR_EXTENSION_METERS, elevator.getElevatorPosition()));
    double pitchAngle = wristPitch.getCurrentPos();
    double rollAngle = wristRoll.getCurrentPos();

    Pose3d userStage1Pose =
        buildUserPose(STAGE1_ZERO_POS, pivotAngle, 0.0, 0.0, new Translation3d());
    Pose3d userStage2Pose =
        buildUserPose(
            STAGE2_ZERO_POS,
            pivotAngle,
            0.0,
            0.0,
            ELEVATOR_EXTENSION_AXIS.times(elevatorExtension));
    Pose3d userStage3Pose =
        buildUserPose(
            STAGE3_ZERO_POS,
            pivotAngle,
            pitchAngle,
            0.0,
            ELEVATOR_EXTENSION_AXIS.times(elevatorExtension));
    Pose3d userStage4Pose =
        buildUserPose(
            STAGE4_ZERO_POS,
            pivotAngle,
            pitchAngle,
            rollAngle,
            ELEVATOR_EXTENSION_AXIS.times(elevatorExtension));

    Logger.recordOutput("Field3d/Coyneinator/Base", Pose3d.struct, basePose);
    Logger.recordOutput(
        "Field3d/Coyneinator/Components/ElevatorStage1", Pose3d.struct, userStage1Pose);
    Logger.recordOutput(
        "Field3d/Coyneinator/Components/ElevatorStage2", Pose3d.struct, userStage2Pose);
    Logger.recordOutput(
        "Field3d/Coyneinator/Components/WristMount", Pose3d.struct, userStage3Pose);
    Logger.recordOutput("Field3d/Coyneinator/Components/Wrist", Pose3d.struct, userStage4Pose);
  }

  private static Pose3d buildUserPose(
      Translation3d zeroPosition,
      double pivotAngle,
      double pitchAngle,
      double rollAngle,
      Translation3d delta) {
    double[][] rotation =
        multiply(
            multiply(rotationY(pivotAngle), rotationY(pitchAngle)),
            rotationX(rollAngle));
    Rotation3d rotation3d = rotationFromMatrix(rotation);
    Translation3d rotatedZero = rotateVector(rotation, zeroPosition);
    Translation3d rotatedDelta = rotateVector(rotation, delta);
    Translation3d translation = zeroPosition.minus(rotatedZero).plus(rotatedDelta);
    return new Pose3d(translation, rotation3d);
  }

  private static double[][] rotationX(double angleRadians) {
    double c = Math.cos(angleRadians);
    double s = Math.sin(angleRadians);
    return new double[][] {
      {1.0, 0.0, 0.0},
      {0.0, c, -s},
      {0.0, s, c}
    };
  }

  private static double[][] rotationY(double angleRadians) {
    double c = Math.cos(angleRadians);
    double s = Math.sin(angleRadians);
    return new double[][] {
      {c, 0.0, s},
      {0.0, 1.0, 0.0},
      {-s, 0.0, c}
    };
  }

  private static double[][] multiply(double[][] a, double[][] b) {
    double[][] result = new double[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result[i][j] =
            a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j];
      }
    }
    return result;
  }

  private static Translation3d rotateVector(double[][] rotation, Translation3d vector) {
    double x =
        rotation[0][0] * vector.getX()
            + rotation[0][1] * vector.getY()
            + rotation[0][2] * vector.getZ();
    double y =
        rotation[1][0] * vector.getX()
            + rotation[1][1] * vector.getY()
            + rotation[1][2] * vector.getZ();
    double z =
        rotation[2][0] * vector.getX()
            + rotation[2][1] * vector.getY()
            + rotation[2][2] * vector.getZ();
    return new Translation3d(x, y, z);
  }

  private static Rotation3d rotationFromMatrix(double[][] matrix) {
    return new Rotation3d(
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            matrix[0][0],
            matrix[0][1],
            matrix[0][2],
            matrix[1][0],
            matrix[1][1],
            matrix[1][2],
            matrix[2][0],
            matrix[2][1],
            matrix[2][2]));
  }
}
