package frc.robot.subsystems;

import java.util.HashMap;

import org.opencv.ml.StatModel;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldLayout;
import frc.robot.StateManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Hexatroller extends SubsystemBase{
    
    private Joystick hexaTroller;

    public static Pose2d currentRobotPoseSelection; // What pose the Spotter has chosen for the robot to go to

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

    private double[] wantedScoringLevel = null;

    private final HashMap<Integer, Pose2d> blueAllianceTags = new HashMap<>();
    private final HashMap<Integer, Pose2d> redAllianceTags = new HashMap<>();
    private HashMap<Integer, Pose2d> selected;

    public Hexatroller(int port) {
        hexaTroller = new Joystick(port);

        blueAllianceTags.put(1, FieldLayout.AprilTags.APRIL_TAG_POSE.get(18).pose.toPose2d());
        blueAllianceTags.put(2, FieldLayout.AprilTags.APRIL_TAG_POSE.get(17).pose.toPose2d());
        blueAllianceTags.put(3, FieldLayout.AprilTags.APRIL_TAG_POSE.get(22).pose.toPose2d());
        blueAllianceTags.put(4, FieldLayout.AprilTags.APRIL_TAG_POSE.get(21).pose.toPose2d());
        blueAllianceTags.put(5, FieldLayout.AprilTags.APRIL_TAG_POSE.get(20).pose.toPose2d());
        blueAllianceTags.put(6, FieldLayout.AprilTags.APRIL_TAG_POSE.get(19).pose.toPose2d());

        //flipped in accordance with unit circle so that operator doesn't crash out while trying to figure out what to click.
        redAllianceTags.put(1, FieldLayout.AprilTags.APRIL_TAG_POSE.get(7).pose.toPose2d()); 
        redAllianceTags.put(2, FieldLayout.AprilTags.APRIL_TAG_POSE.get(8).pose.toPose2d());
        redAllianceTags.put(3, FieldLayout.AprilTags.APRIL_TAG_POSE.get(9).pose.toPose2d());
        redAllianceTags.put(4, FieldLayout.AprilTags.APRIL_TAG_POSE.get(10).pose.toPose2d());
        redAllianceTags.put(5, FieldLayout.AprilTags.APRIL_TAG_POSE.get(11).pose.toPose2d());
        redAllianceTags.put(6, FieldLayout.AprilTags.APRIL_TAG_POSE.get(6).pose.toPose2d());

    }

    public static Pose2d getRequestedPosition() {
        //the robot will go up 6.5 and left or right 6.5 inches (the hexagon creates an equilateral triangle)
        return currentRobotPoseSelection;
    }

    public Pose2d chosenAprilTag(ScoringPosition wanted) {

        if (DriverStation.getAlliance().orElse(null) == Alliance.Red) {
            selected = redAllianceTags;
        } else {
            selected = blueAllianceTags;
        }

        Pose2d chosenOffset = null;

        switch (wanted) {
            case A:
            case B:
                chosenOffset = selected.get(1);
                break;
            case C:
            case D:
                chosenOffset = selected.get(2);
                break;
            case E:
            case F:
                return selected.get(3);

            case G:
            case H:
                chosenOffset = selected.get(4);
                break;
            case I:
            case J:
                chosenOffset = selected.get(5);
                break;
            case K:
            case L:
                chosenOffset = selected.get(6);
                break;

        }

        return chosenOffset;

    }

    public ScoringPosition selectPosition(int pressedButton) {
        //offset 1.1 feet apart from center
        // offset will be 13 inches / 2 for each side (so to sscore in the coral the robot will have to move 6.5 inches to the left or to the right)

        switch (pressedButton) {

            case 1:
                chosen = ScoringPosition.A;
                break;
            case 2:
                chosen = ScoringPosition.B;
                break;
            case 3:
                chosen = ScoringPosition.C;
                break;
            case 4:
                chosen = ScoringPosition.D;
                break;
            case 5:
                chosen = ScoringPosition.E;
                break;
            case 6:
                chosen = ScoringPosition.F;
                break;
            case 7:
                chosen = ScoringPosition.G;
                break;
            case 8:
                chosen = ScoringPosition.H;
                break;
            case 9:
                chosen = ScoringPosition.I;
                break;
            case 10:
                chosen = ScoringPosition.J;
                break;
            case 11:
                chosen = ScoringPosition.K;
                break;
            case 12:
                chosen = ScoringPosition.L;
                break;
        }

        return chosen;
    }

    public Pose2d offset(ScoringPosition wanted) {

        Pose2d chosenOffset = new Pose2d();

        switch (wanted) {
            case A:
                chosenOffset = new Pose2d(0.0, Units.inchesToMeters(6.5), new Rotation2d());
                break;
            case B:
                chosenOffset = new Pose2d(0.0, Units.inchesToMeters(-6.5), new Rotation2d());
                break;
            case C:
                chosenOffset = new Pose2d(Units.inchesToMeters(-1 * (6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(6.5 / 2), new Rotation2d());
                break;
            case D:
                chosenOffset = new Pose2d(Units.inchesToMeters((6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(-1 * (6.5 / 2)), new Rotation2d());
                break;
            case E:
                chosenOffset = new Pose2d(Units.inchesToMeters(-1 * (6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(6.5 / 2), new Rotation2d());
                break;
            case F:
                chosenOffset = new Pose2d(Units.inchesToMeters((6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(-1 * (6.5 / 2)), new Rotation2d());
                break;
            case G:
                chosenOffset = new Pose2d(0, Units.inchesToMeters(-6.5), new Rotation2d());
                break;
            case H:
                chosenOffset = new Pose2d(0, Units.inchesToMeters(6.5), new Rotation2d());
                break;
            case I:
                chosenOffset = new Pose2d(Units.inchesToMeters((6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(-1 * (6.5 / 2)), new Rotation2d());
                break;
            case J:
                chosenOffset = new Pose2d(Units.inchesToMeters(-1 * ((6.5 / 2) * Math.sqrt(3))), Units.inchesToMeters(6.5 / 2), new Rotation2d());
                break;
            case K:
                chosenOffset = new Pose2d(Units.inchesToMeters((6.5 / 2) * Math.sqrt(3)), Units.inchesToMeters(6.5 / 2), new Rotation2d());
                break;
            case L:
                chosenOffset = new Pose2d(Units.inchesToMeters(-1 * ((6.5 / 2) * Math.sqrt(3))), Units.inchesToMeters(-1 * (6.5 / 2)), new Rotation2d());
                break;


        }   

        return chosenOffset;
    }

    public Pose2d robotPoseOffset(ScoringPosition wanted) {

        Pose2d chosenOffset = null;

        switch (wanted) {

            case A:
            case B:
                chosenOffset = new Pose2d(0.0, Units.inchesToMeters(-14), new Rotation2d(Units.degreesToRadians(0)));
                break;


            case C:
            case D:
                chosenOffset = new Pose2d(Units.inchesToMeters(14 * Math.sin(225)), Units.inchesToMeters(14 * Math.cos(225)), new Rotation2d(Units.degreesToRadians(60)));
                break;


            case E:
            case F:
                chosenOffset = new Pose2d(Units.inchesToMeters(14 * Math.sin(315)), Units.inchesToMeters(14 * Math.cos(315)), new Rotation2d(Units.degreesToRadians(120)));
                break;


            case G:
            case H:
                chosenOffset = new Pose2d(0.0, Units.inchesToMeters(14), new Rotation2d(Units.degreesToRadians(180)));
                break;


            case I:
            case J:
                chosenOffset = new Pose2d(Units.inchesToMeters(14 * Math.sin(45)), Units.inchesToMeters(14 * Math.cos(45)), new Rotation2d(Units.degreesToRadians(240)));
                break;


            case K:
            case L:
                chosenOffset = new Pose2d(Units.inchesToMeters(14 * Math.sin(135)), Units.inchesToMeters(14 * Math.cos(135)), new Rotation2d(Units.degreesToRadians(300)));
                break;

        }

        return chosenOffset;
    }


    public boolean isButtonPressed(int buttonNumber) {
        return hexaTroller.getRawButton(buttonNumber);
    }

    public double[] getWantedScoringLevel(int buttonNumber) {

        double[] wantedScoringLevel = null;

        switch(buttonNumber) {
            case 13:
                wantedScoringLevel = StateManager.getCoordinate(StateManager.Positions.L1);
                break;
            case 14:
                wantedScoringLevel = StateManager.getCoordinate(StateManager.Positions.L2);
                break;
            case 15:
                wantedScoringLevel = StateManager.getCoordinate(StateManager.Positions.L3);
                break;
            case 16:
                wantedScoringLevel = StateManager.getCoordinate(StateManager.Positions.L4);
                break;
        }

        return wantedScoringLevel;

    }

    public double[] whereToScore() {
        return wantedScoringLevel;
    }

    @Override
    public void periodic() {

        for (int i = 1; i <= 12; i++) {
            if (isButtonPressed(i) && StateManager.getCurrentState() == StateManager.RobotStateMachine.Coral) {

                currentRobotPoseSelection = new Pose2d(chosenAprilTag(selectPosition(i)).getX() + offset(selectPosition(i)).getX() + robotPoseOffset(selectPosition(i)).getX(), 
                                                       chosenAprilTag(selectPosition(i)).getY() + offset(selectPosition(i)).getY() + robotPoseOffset(selectPosition(i)).getY(), 
                                                       robotPoseOffset(selectPosition(i)).getRotation());

            } else if (isButtonPressed(i) && StateManager.getCurrentState() == StateManager.RobotStateMachine.Algae) {

                currentRobotPoseSelection = new Pose2d(chosenAprilTag(selectPosition(i)).getX() + robotPoseOffset(selectPosition(i)).getX(), 
                                                       chosenAprilTag(selectPosition(i)).getY() + robotPoseOffset(selectPosition(i)).getY(), 
                                                       robotPoseOffset(selectPosition(i)).getRotation());
            }
        }


        for (int i = 13; i <= 16; i++) {
            if (isButtonPressed(i)) {
                wantedScoringLevel = getWantedScoringLevel(i);
            }
        }
    }

}