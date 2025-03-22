// TODO: V This is definitely wrong, but I think it's a good framework for a simple swerve move just to get started? V

package frc.robot.autos.autocommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class testAuto extends Command {
    private final Swerve s_Swerve;

    public testAuto(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        // Initialization logic if needed
    }

    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(10, 0), 180, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, false, false); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        return true; // Change this if you need continuous movement
    }
}
