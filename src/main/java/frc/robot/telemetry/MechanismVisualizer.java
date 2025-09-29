package frc.robot.telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lightweight visualizer that publishes Mechanism2d views for the arm stack
 * (pivot + elevator + wrist pitch/roll) and the swerve module azimuths.
 *
 * Published topics (AdvantageKit):
 *  - Mechanisms/Arm2D
 *  - Mechanisms/Swerve2D
 */
public class MechanismVisualizer extends SubsystemBase {
    // Suppliers for arm state
    private final DoubleSupplier pivotRad;
    private final DoubleSupplier elevatorMeters;
    private final DoubleSupplier wristPitchRad;
    private final DoubleSupplier wristRollRad;

    // Suppliers for swerve state
    private final Supplier<SwerveModuleState[]> moduleStatesSupplier;
    private final Supplier<Rotation2d> headingSupplier;

    // Arm visualization
    private final LoggedMechanism2d armMech = new LoggedMechanism2d(300, 220);
    private final LoggedMechanismRoot2d armBase = armMech.getRoot("Base", 150, 20);
    private final LoggedMechanismLigament2d pivotLig = armBase.append(new LoggedMechanismLigament2d("Pivot", 40, 90));
    private final LoggedMechanismLigament2d elevatorLig = pivotLig.append(new LoggedMechanismLigament2d("Elevator", 60, 0));
    private final LoggedMechanismLigament2d wristPitchLig = elevatorLig.append(new LoggedMechanismLigament2d("WristPitch", 25, 0));
    private final LoggedMechanismLigament2d wristRollLig = wristPitchLig.append(new LoggedMechanismLigament2d("WristRoll", 15, 0));

    // Swerve visualization
    private final LoggedMechanism2d swerveMech = new LoggedMechanism2d(300, 220);
    private final LoggedMechanismRoot2d flRoot = swerveMech.getRoot("FL", 80, 160);
    private final LoggedMechanismRoot2d frRoot = swerveMech.getRoot("FR", 220, 160);
    private final LoggedMechanismRoot2d blRoot = swerveMech.getRoot("BL", 80, 60);
    private final LoggedMechanismRoot2d brRoot = swerveMech.getRoot("BR", 220, 60);
    private final LoggedMechanismLigament2d flAz = flRoot.append(new LoggedMechanismLigament2d("FL_Az", 25, 0));
    private final LoggedMechanismLigament2d frAz = frRoot.append(new LoggedMechanismLigament2d("FR_Az", 25, 0));
    private final LoggedMechanismLigament2d blAz = blRoot.append(new LoggedMechanismLigament2d("BL_Az", 25, 0));
    private final LoggedMechanismLigament2d brAz = brRoot.append(new LoggedMechanismLigament2d("BR_Az", 25, 0));

    // Scaling constants
    private static final double ELEVATOR_METERS_TO_PIXELS = 120.0; // tune to taste

    public MechanismVisualizer(
        DoubleSupplier pivotRad,
        DoubleSupplier elevatorMeters,
        DoubleSupplier wristPitchRad,
        DoubleSupplier wristRollRad,
        Supplier<SwerveModuleState[]> moduleStatesSupplier,
        Supplier<Rotation2d> headingSupplier
    ) {
        this.pivotRad = pivotRad;
        this.elevatorMeters = elevatorMeters;
        this.wristPitchRad = wristPitchRad;
        this.wristRollRad = wristRollRad;
        this.moduleStatesSupplier = moduleStatesSupplier;
        this.headingSupplier = headingSupplier;
    }

    @Override
    public void periodic() {
        // Update arm stack
        double pivotDeg = Math.toDegrees(pivotRad.getAsDouble());
        double elevLenPx = Math.max(0.0, 40.0 + elevatorMeters.getAsDouble() * ELEVATOR_METERS_TO_PIXELS);
        double wristPitchDeg = Math.toDegrees(wristPitchRad.getAsDouble());
        double wristRollDeg = Math.toDegrees(wristRollRad.getAsDouble());

        pivotLig.setAngle(90.0 + pivotDeg); // 90° = straight up
        elevatorLig.setLength(elevLenPx);
        elevatorLig.setAngle(0.0); // extend along pivot direction
        wristPitchLig.setAngle(wristPitchDeg);
        wristRollLig.setAngle(wristRollDeg);

        Logger.recordOutput("Mechanisms/Arm2D", armMech);

        // Update swerve module azimuths
        SwerveModuleState[] states = moduleStatesSupplier.get();
        if (states != null && states.length >= 4) {
            flAz.setAngle(Math.toDegrees(states[0].angle.getRadians()));
            frAz.setAngle(Math.toDegrees(states[1].angle.getRadians()));
            blAz.setAngle(Math.toDegrees(states[2].angle.getRadians()));
            brAz.setAngle(Math.toDegrees(states[3].angle.getRadians()));

            // Optionally scale ligament length with speed for a quick visual cue
            flAz.setLength(15 + 10 * Math.abs(states[0].speedMetersPerSecond));
            frAz.setLength(15 + 10 * Math.abs(states[1].speedMetersPerSecond));
            blAz.setLength(15 + 10 * Math.abs(states[2].speedMetersPerSecond));
            brAz.setLength(15 + 10 * Math.abs(states[3].speedMetersPerSecond));
        }

        Logger.recordOutput("Mechanisms/Swerve2D", swerveMech);
    }
}
