package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class FieldLayout {

    public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317);

    public static final double startingLineX =
      Units.inchesToMeters(299.438);


    public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }
    
    public static class AprilTags {
        
        /* Degrees face the red alliance drivestations at 0 radians. 
         * View https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
         * for more information
        */
        public static final List<AprilTag> APRIL_TAG_POSE = new ArrayList<AprilTag>(Arrays.asList(
            new AprilTag(1,
                new Pose3d(
                    Units.inchesToMeters(657.37),
                    Units.inchesToMeters(25.80),
                    Units.inchesToMeters(58.50),
                    new Rotation3d(0, 0, Units.degreesToRadians(126))
                )
            ),
            new AprilTag(
                2,
                new Pose3d(
                    Units.inchesToMeters(657.37),
                    Units.inchesToMeters(291.20),
                    Units.inchesToMeters(58.50),
                    new Rotation3d(0, 0, Units.degreesToRadians(234))
                )
            ),
            new AprilTag(
                3,
                new Pose3d(
                    Units.inchesToMeters(455.15 ),
                    Units.inchesToMeters(317.15 ),
                    Units.inchesToMeters(51.25),
                    new Rotation3d(0, 0, Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                4,
                new Pose3d(
                    Units.inchesToMeters(365.20),
                    Units.inchesToMeters(241.64),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, 0, Units.degreesToRadians(30))
                )
            ),
            new AprilTag(
                5,
                new Pose3d(
                    Units.inchesToMeters(365.20),
                    Units.inchesToMeters(75.39),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, 0, Units.degreesToRadians(30))
                )
            ),
            new AprilTag(
                6,
                new Pose3d(
                    Units.inchesToMeters(530.49),
                    Units.inchesToMeters(130.17),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, Units.degreesToRadians(300), Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                7,
                new Pose3d(
                    Units.inchesToMeters(546.87),
                    Units.inchesToMeters(158.50),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 0)
                )
            ),
            new AprilTag(
                8,
                new Pose3d(
                    Units.inchesToMeters(530.49),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 60 )
                )
            ),
            new AprilTag(
                9,
                new Pose3d(
                    Units.inchesToMeters(497.77 ),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13 ),
                    new Rotation3d(0, 0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                10,
                new Pose3d(
                    Units.inchesToMeters(481.39),
                    Units.inchesToMeters(158.50),
                    Units.inchesToMeters(12.13 ),
                    new Rotation3d(0, 0, Units.degreesToRadians(180))
                )
            ),
            new AprilTag(11,
            new Pose3d(
                    Units.inchesToMeters(497.77),
                    Units.inchesToMeters(130.17),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(240))
                )
            ),
            new AprilTag(
                12,
                new Pose3d(
                    Units.inchesToMeters(33.51),
                    Units.inchesToMeters(25.80),
                    Units.inchesToMeters(58.50),
                    new Rotation3d(0, 0, Units.degreesToRadians(54))
                )
            ),
            new AprilTag(
                13,
                new Pose3d(
                    Units.inchesToMeters(33.51 ),
                    Units.inchesToMeters(291.20),
                    Units.inchesToMeters(58.50),
                    new Rotation3d(0, 0, Units.degreesToRadians(306))
                )
            ),
            new AprilTag(
                14,
                new Pose3d(
                    Units.inchesToMeters(325.68),
                    Units.inchesToMeters(241.64),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180))
                )
            ),
            new AprilTag(
                15,
                new Pose3d(
                    Units.inchesToMeters(325.68),
                    Units.inchesToMeters(75.39),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180))
                )
            ),
            new AprilTag(
                16,
                new Pose3d(
                    Units.inchesToMeters(235.73),
                    Units.inchesToMeters(-0.15 ),
                    Units.inchesToMeters(51.25),
                    new Rotation3d(0, 0, Units.degreesToRadians(90))
                       
                )
            ),

            new AprilTag(
                17, 
                new Pose3d(
                    Units.inchesToMeters(160.39),
                    Units.inchesToMeters(130.17),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(240))
                )
            ),

            new AprilTag(
                18, 
                new Pose3d(
                    Units.inchesToMeters(144.00),
                    Units.inchesToMeters(158.50),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(180))
                )
            ),

            new AprilTag(
                19, 
                new Pose3d(
                    Units.inchesToMeters(160.39),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(120))
                )
            ),

            new AprilTag(
                20, 
                new Pose3d(
                    Units.inchesToMeters(193.10),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(60))
                )
            ),

            new AprilTag(
                21, 
                new Pose3d(
                    Units.inchesToMeters(209.49),
                    Units.inchesToMeters(158.50),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 0)
                )
            ),

            new AprilTag(
                22, 
                new Pose3d(
                    Units.inchesToMeters(193.10),
                    Units.inchesToMeters(130.17),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, Units.degreesToRadians(300))
                )
            )
        ));
    }



    public static class Reef {

    public static final char[] branches = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
    public static Map<Character, Integer> branchesToInt = new HashMap<>();
    public static Map<Character, Integer> branchesToAprilTagID = new HashMap<>();


    public static final double reefdFacesWidth = Units.inchesToMeters(30.27);
    public static final double reefdFacesHeight = Units.inchesToMeters(37.27);

    public static final double reefHitboxRadius = Units.inchesToMeters(83); //TODO tune this value if it doesn't work well

    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {

        
    branchesToInt.put('A', 1);
    branchesToInt.put('B', 2);
    branchesToInt.put('C', 3);
    branchesToInt.put('D', 4);
    branchesToInt.put('E', 5);
    branchesToInt.put('F', 6);
    branchesToInt.put('G', 7);
    branchesToInt.put('H', 8);
    branchesToInt.put('I', 9);
    branchesToInt.put('J', 10);
    branchesToInt.put('K', 11);
    branchesToInt.put('L', 12);

    branchesToAprilTagID.put('A', 18);
    branchesToAprilTagID.put('B', 18);
    branchesToAprilTagID.put('C', 17);
    branchesToAprilTagID.put('D', 17);
    branchesToAprilTagID.put('E', 22);
    branchesToAprilTagID.put('F', 22);
    branchesToAprilTagID.put('G', 21);
    branchesToAprilTagID.put('H', 21);
    branchesToAprilTagID.put('I', 20);
    branchesToAprilTagID.put('J', 20);
    branchesToAprilTagID.put('K', 19);
    branchesToAprilTagID.put('L', 19);
  }
}

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }
}