package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

public class Hexatroller {
    
    private Joystick hexaTroller;

    private enum ScoringPosition {
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

    private ScoringPosition chosen = null;

    private final HashMap<ScoringPosition, Double> coral = new HashMap<>();
    private final HashMap<ScoringPosition, Double> algae = new HashMap<>();
    private HashMap<ScoringPosition, Double> selected;

    public Hexatroller(int port) {
        hexaTroller = new Joystick(port);

    }

    public void WhereToGo() {
        //the robot will go up 6.5 and left or right 6.5 inches (the hexagon creates an equilateral triangle)

        


    }

    public ScoringPosition switchWhereRobotWillScore(int pressedButton) {
        //offset 1.1 feet apart from center
        // offset will be 13 inches / 2 for each side (so to sscore in the coral the robot will have to move 6.5 inches to the left or to the right)

        switch (pressedButton) {

            case 1:
                chosen = ScoringPosition.A;
            case 2:
                chosen = ScoringPosition.B;
            case 3:
                chosen = ScoringPosition.C;
            case 4:
                chosen = ScoringPosition.D;
            case 5:
                chosen = ScoringPosition.E;
            case 6:
                chosen = ScoringPosition.F;
            case 7:
                chosen = ScoringPosition.G;
            case 8:
                chosen = ScoringPosition.H;
            case 9:
                chosen = ScoringPosition.I;
            case 10:
                chosen = ScoringPosition.J;
            case 11:
                chosen = ScoringPosition.K;
            case 12:
                chosen = ScoringPosition.L;
        }

        return chosen;
    }

    public Pose2d Offset(ScoringPosition wanted) {

        Pose2d chosenOffset = new Pose2d();

        switch (wanted) {
            case A:
                chosenOffset = new Pose2d(6.5, 0.0, new Rotation2d());

            case B:
                chosenOffset = new Pose2d(-6.5 , 0.0, new Rotation2d());
                
            case C:
                chosenOffset = new Pose2d(6.5 / 2, -1 * (6.5 / 2) * Math.sqrt(3), new Rotation2d()); //We might not need to do this we could just flip maybe?

            case D:
                chosenOffset = new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case E:
                chosenOffset = new Pose2d(6.5 / 2, -1 * (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case F:
                chosenOffset = new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case G:
                chosenOffset = new Pose2d(-6.5, 0, new Rotation2d());
            case H:
                chosenOffset = new Pose2d( 6.5, 0, new Rotation2d());
            case I:
                chosenOffset = new Pose2d( -1 * (6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case J:
                chosenOffset = new Pose2d(6.5 / 2, -1 * ((6.5 / 2) * Math.sqrt(3)), new Rotation2d());
            case K:
                chosenOffset = new Pose2d((6.5 / 2), (6.5 / 2) * Math.sqrt(3), new Rotation2d());
            case L:
                chosenOffset = new Pose2d( -1 * (6.5 / 2), -1 * ((6.5 / 2) * Math.sqrt(3)), new Rotation2d());


        }

        return chosenOffset;
    }
}
