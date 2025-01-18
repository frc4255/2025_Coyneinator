package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ScoringPositions;
import frc.robot.Constants.ScoringPositions.setpoints;
import frc.robot.subsystems.Hexatroller;

public class OffsetCalculator {
    
    private enum Hexagon {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L, 
    };

    public OffsetCalculator() {


    }

    public Pose2d test(Hexagon requested) {

        if (requested == A) {

        }


    }

    public Pose2d Offset(Hexagon wanted) {

        Pose2d chosenOffset = new Pose2d();

        switch (wanted) {
            case A:
                return new Pose2d(6.5, 0.0, new Rotation2d());

            case B:
                return new Pose2d(-6.5 , 0.0, new Rotation2d());
                
            case C:
                return new Pose2d(6.5 / 2, -1 * (6.5 / 2) * Math.sqrt(3), new Rotation2d()); //We might not need to do this we could just flip maybe?

            case D:
                return new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case E:
                return new Pose2d(6.5 / 2, -1 * (6.5 / 2) * Math.sqrt(3), new Rotation2d());

            case F:
                return new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case G:
                return new Pose2d(-6.5, 0, new Rotation2d());
            case H:
                return new Pose2d( 6.5, 0, new Rotation2d());
            case I:
                return new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case J:
                return new Pose2d(6.5 / 2, -1 * ((6.5 / 2) * Math.sqrt(3)), new Rotation2d());
            case K:
                return new Pose2d((6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case L:
                return new Pose2d( -1 * (6.5 / 2), -1 * ((6.5 / 2) * Math.sqrt(3)), new Rotation2d());


        }

        return new Pose2d();
    }
}
