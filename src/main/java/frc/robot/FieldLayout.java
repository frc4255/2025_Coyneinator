package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class FieldLayout {

    public static final double FIELD_LENGTH = 16.451;
    public static final double FIELD_WIDTH = 8.211;

    public static final double BLUE_WING_LINE = 231.2;
    public static final double RED_WING_LINE = 422.01;
    public static class FieldPiece {

        /* We only care about the speakers for autonomous controls. */
        public static enum POI {
            BLUE_SPEAKER,
            RED_SPEAKER
        }

        public static final Map<POI, Pose3d> POI_POSE =
            Map.of(
                POI.BLUE_SPEAKER,
                new Pose3d(
                    0.0,
                    Units.inchesToMeters(224.42),
                    Units.inchesToMeters(2.04216),
                    new Rotation3d(0, 0, 0)
                ),
                POI.RED_SPEAKER,
                new Pose3d(
                    Units.inchesToMeters(652.73),
                    Units.inchesToMeters(220.42),
                    Units.inchesToMeters(2.04216),
                    new Rotation3d(0, 0, Math.PI)
                )
            );
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
                    new Rotation3d(0, 0, 30)
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
                    new Rotation3d(0, 300, Units.degreesToRadians(270))
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
                    new Rotation3d(0, 0, 306)
                )
            ),
            new AprilTag(
                14,
                new Pose3d(
                    Units.inchesToMeters(325.68),
                    Units.inchesToMeters(241.64),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, 30, 180)
                )
            ),
            new AprilTag(
                15,
                new Pose3d(
                    Units.inchesToMeters(325.68),
                    Units.inchesToMeters(75.39),
                    Units.inchesToMeters(73.54),
                    new Rotation3d(0, 30, Units.degreesToRadians(180))
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
                    new Rotation3d(0, 0, 240)
                )
            ),

            new AprilTag(
                18, 
                new Pose3d(
                    Units.inchesToMeters(144.00),
                    Units.inchesToMeters(158.50),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 180)
                )
            ),

            new AprilTag(
                19, 
                new Pose3d(
                    Units.inchesToMeters(160.39),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 120)
                )
            ),

            new AprilTag(
                20, 
                new Pose3d(
                    Units.inchesToMeters(193.10),
                    Units.inchesToMeters(186.83),
                    Units.inchesToMeters(12.13),
                    new Rotation3d(0, 0, 60)
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
                    new Rotation3d(0, 0, 300)
                )
            )
        ));
    }
}