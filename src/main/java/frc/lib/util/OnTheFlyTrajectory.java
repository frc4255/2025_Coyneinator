package frc.lib.util;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;
import frc.robot.FieldLayout;
import frc.robot.subsystems.AlignTool;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

public class OnTheFlyTrajectory {

    private Swerve s_Swerve;

    private ArrayList<Pose2d> reefFaces = new ArrayList<Pose2d>();

    private double reefRadius = FieldLayout.Reef.reefHitboxRadius;
    private Translation2d reefCenter;

    public OnTheFlyTrajectory(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;

        for (Pose2d reef : FieldLayout.Reef.centerFaces) {
            reefFaces.add(reef);
        }

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

            Pose2d aprilTag1 = FieldLayout.AprilTags.APRIL_TAG_POSE.get(10).pose.toPose2d();
            Pose2d aprilTag2 = FieldLayout.AprilTags.APRIL_TAG_POSE.get(7).pose.toPose2d();
             
            reefCenter = new Translation2d((aprilTag2.getX() + aprilTag1.getX()) / 2, 
                                            (aprilTag2.getY() + aprilTag1.getY()) / 2);

        } else {
            reefCenter = FieldLayout.Reef.center;
        }
    }


    
    public PathPlannerPath newOnTheFlyPath(Pose2d endPose) {
        Pose2d currentPose = s_Swerve.getPose();
        Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPose,                           
                endPose                            
        );

        // Define the path constraints
        PathConstraints constraints = new PathConstraints(
                3.0,         
                3.0,        
                2 * Math.PI, 
                4 * Math.PI 
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, endPose.getRotation()) // Goal end state (can set holonomic rotation for swerve)
        );

        path.preventFlipping = true;

        return path;
    }


    public boolean doesPathGoesThroughReef(PathPlannerPath path) {

        if (path == null || path.getPathPoses() == null || path.getPathPoses().isEmpty()) {
            System.out.println("Path is invalid or null. Exiting function.");
            return false; 
        }

        for (Pose2d point : path.getPathPoses()) {
            if (Math.pow(point.getX() - reefCenter.getX(), 2) + 
                Math.pow(point.getY() - reefCenter.getY(), 2) <= Math.pow(reefRadius, 2)) {
                return true;
            }
        }
        return false;
    }

    public PathPlannerPath generatePathWithAvoidance(Pose2d endPose) {
        Pose2d startPose = s_Swerve.getPose();
        
        // Create initial path
        PathPlannerPath path = newOnTheFlyPath(endPose);
    
        if (!doesPathGoesThroughReef(path)) {
            return path; 
        } else {
            calculateAvoidanceWaypoints(startPose, endPose);
        }
    
        List<Waypoint> waypoints = calculateAvoidanceWaypoints(startPose, endPose);
    
        PathConstraints constraints = new PathConstraints(
            3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    

        PathPlannerPath newPath = new PathPlannerPath(
            waypoints,
            constraints,
            null, 
            new GoalEndState(0.0, endPose.getRotation()) 
        );
    
        newPath.preventFlipping = true; 
    
        return newPath;
    }
    
    private List<Waypoint> calculateAvoidanceWaypoints(Pose2d start, Pose2d end) {
        List<Waypoint> waypoints = new ArrayList<>();
    
        double midX = (start.getX() + end.getX()) / 2.0;
        double midY = (start.getY() + end.getY()) / 2.0;
        Translation2d midPoint = new Translation2d(midX, midY);
    
        double dx = midPoint.getX() - reefCenter.getX();
        double dy = midPoint.getY() - reefCenter.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
    
        double detourDistance = reefRadius + 5; //Offset tune as needed
        double offsetX = -dy / distance * detourDistance;
        double offsetY = dx / distance * detourDistance;
    
        Translation2d detourAnchor = new Translation2d(midX + offsetX, midY + offsetY);
        Waypoint detourWaypoint = new Waypoint(
            detourAnchor.minus(new Translation2d(0.2, 0)), 
            detourAnchor, 
            detourAnchor.plus(new Translation2d(0.2, 0))
        );
    
        waypoints.add(new Waypoint(start.getTranslation(), start.getTranslation(), start.getTranslation()));
        waypoints.add(detourWaypoint);
        waypoints.add(new Waypoint(end.getTranslation(), end.getTranslation(), end.getTranslation()));
    
        return waypoints;
    }
    
}