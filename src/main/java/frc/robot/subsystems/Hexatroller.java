package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
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

    private final HashMap<ScoringPosition, Double> Coral = new HashMap<>();
  private final HashMap<ScoringPosition, Double> algae = new HashMap<>();
  private HashMap<ScoringPosition, Double> selected;

    public Hexatroller(int port) {
        hexaTroller = new Joystick(port);

    }

    public void WhereToGo() {
        //the robot will go up 6.5 and left or right 6.5 inches (the hexagon creates an equilateral triangle)



    }

    public void switchWhereRobotWillScore(int pressedButton) {
        //offset 1.1 feet apart from center
        // offset will be 13 inches / 2 for each side (so to sscore in the coral the robot will have to move 6.5 inches to the left or to the right)

        Pose2d goal = new Pose2d();

        switch (pressedButton) {

            case 1:
                goal = new Pose2d();
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
        }

    }
}
