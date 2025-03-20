package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Camera;

public class L1Assist extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;
    private Swerve s_Swerve;

    private List<Camera> cams;
    
    private char sector;

    public L1Assist(SubsystemManager manager, Swerve swerve) {
        this.manager = manager;
        this.s_Swerve = swerve;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        //TODO: Insert try catch here bc we don't want stupid code to make us die
        //dector = s_Swerve.getCurrentBranchSector(DriverStation.getAlliance().get() == Alliance.Red ? true : false);
        manager.requestNode(GraphParser.getNodeByName("Reef Align"));
    }

    @Override
    public void execute() {
    }
}
