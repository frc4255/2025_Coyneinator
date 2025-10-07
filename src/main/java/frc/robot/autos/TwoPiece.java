package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Swerve swerve, TrajectoryLibrary trajectories){
        addCommands(trajectories.buildFollowPathCommand("2 Piece", swerve));
    }
}
