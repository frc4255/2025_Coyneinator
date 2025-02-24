package frc.lib.util;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;
import frc.robot.FieldLayout;
import frc.robot.subsystems.Hexatroller;

public class OnTheFlyTrajectory {

    private Swerve s_Swerve;

    private ArrayList<Pose2d> reefFaces = new ArrayList<Pose2d>();

    public OnTheFlyTrajectory(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;

        for (Pose2d reef : FieldLayout.Reef.centerFaces) {
            reefFaces.add(reef);
        }
    }


    public PathPlannerPath newOnTheFlyPath() {
        Pose2d currentPose = s_Swerve.getPose();
        Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation());


        Pose2d endPose = Hexatroller.getRequestedPosition(); 

        // Define additional waypoints between the start and end positions (optional)
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
                null, // Ideal starting state, not needed for on-the-fly paths
                new GoalEndState(0.0, Hexatroller.getRequestedPosition().getRotation()) // Goal end state (can set holonomic rotation for swerve)
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

}
