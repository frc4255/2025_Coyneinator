package frc.robot.autos;

import frc.robot.Constants;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Swerve s_Swerve){
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("2 Piece");
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            Alert alert = new Alert("Auto Path not found this would be really bad to see during comp Rad & Nick", AlertType.kError);
            alert.set(true);
        }
        
        final PathPlannerPath s_path = path;

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if (s_path == null) {
            Alert alert = new Alert("Auto Path not found this would be really bad to see during comp Rad & Nick", AlertType.kError);
            alert.set(true);
        } else {
            addCommands(
                new InstantCommand(() -> s_Swerve.setPose(DriverStation.getAlliance().get() == Alliance.Red ?
                    s_path.flipPath().getStartingDifferentialPose(): s_path.getStartingDifferentialPose()))

                //s_Swerve.followPathCommand(s_path)
            ); 
        }   
    }
}