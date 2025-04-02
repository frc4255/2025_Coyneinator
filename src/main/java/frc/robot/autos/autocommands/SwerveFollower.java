package frc.robot.autos.autocommands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveFollower extends Command{
    private Swerve s_Swerve;
    private Trajectory<SwerveSample> traj;
    private final Timer timer = new Timer();

    public SwerveFollower(Swerve s_Swerve, Trajectory<SwerveSample> traj) {
        this.s_Swerve = s_Swerve;
        this.traj = traj;

    }

    @Override
    public void initialize() {
        timer.restart();

    }

    @Override
    public void execute() {
        SwerveSample sample = traj.sampleAt(timer.get(), DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)).get();

        s_Swerve.followTrajectory(sample);
    }
}
