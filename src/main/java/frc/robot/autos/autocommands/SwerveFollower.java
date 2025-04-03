package frc.robot.autos.autocommands;

import java.util.Optional;

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
    private DriverStation.Alliance alliance;

    public SwerveFollower(Swerve s_Swerve, Trajectory<SwerveSample> traj) {
        this.s_Swerve = s_Swerve;
        this.traj = traj;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        alliance = DriverStation.getAlliance().get();
        timer.restart();
    }

    @Override
    public void execute() {
        Optional<SwerveSample> sample = traj.sampleAt(timer.get(), (alliance == Alliance.Red ? true : false));

        if (sample.isPresent()) {
            s_Swerve.followTrajectory(sample.get());
        }
    }
}
