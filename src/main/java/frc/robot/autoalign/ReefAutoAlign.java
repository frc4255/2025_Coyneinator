package frc.robot.autoalign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.Branch;
import org.littletonrobotics.junction.Logger;

/**
 * Backend helper that computes reef-alignment targets and simple PID outputs.
 *
 * <p>The calculate method determines which reef sector (branch) the robot currently occupies, looks
 * up the associated scoring pose, and produces field- and robot-relative chassis speeds using three
 * independent PID controllers. Driver commands can wrap this helper to implement automated reef
 * alignment without re-creating targeting logic.</p>
 */
public final class ReefAutoAlign {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private final double maxLinearSpeedMetersPerSecond;
    private final double maxAngularSpeedRadPerSecond;
    private double standoffMeters;

    public ReefAutoAlign() {
        this(2.5, 2.5, 3.0, 3.0, Math.PI);
    }

    public ReefAutoAlign(
            double kPx,
            double kPy,
            double kPTheta,
            double maxLinearSpeedMetersPerSecond,
            double maxAngularSpeedRadPerSecond
    ) {
        xController = new PIDController(kPx, 0.0, 0.0);
        yController = new PIDController(kPy, 0.0, 0.0);
        thetaController = new PIDController(kPTheta, 0.0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.maxLinearSpeedMetersPerSecond = maxLinearSpeedMetersPerSecond;
        this.maxAngularSpeedRadPerSecond = maxAngularSpeedRadPerSecond;
        this.standoffMeters = edu.wpi.first.math.util.Units.inchesToMeters(-20.0); // default back-off from reef
    }

    public void setStandoffMeters(double meters) {
        this.standoffMeters = meters;
    }

    public double getStandoffMeters() {
        return standoffMeters;
    }

    public AlignmentResult calculate(Pose2d currentPose, boolean isRedAlliance) {
        Branch branch = FieldLayout.getClosestBranch(currentPose, isRedAlliance);
        int sectorIndex = branch.ordinal() + 1;

        Pose2d targetPose = FieldLayout.getRobotApproachPose(branch, isRedAlliance, standoffMeters);

        Logger.recordOutput("AutoAlign/ReefSector", sectorIndex);
        Logger.recordOutput("AutoAlign/ReefBranchIndex", branch.ordinal());
        Logger.recordOutput("AutoAlign/ReefTargetPose", targetPose);
        Logger.recordOutput("AutoAlign/StandoffMeters", standoffMeters);

        double vx = xController.calculate(currentPose.getX(), targetPose.getX());
        double vy = yController.calculate(currentPose.getY(), targetPose.getY());
        double omega = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        vx = MathUtil.clamp(vx, -maxLinearSpeedMetersPerSecond, maxLinearSpeedMetersPerSecond);
        vy = MathUtil.clamp(vy, -maxLinearSpeedMetersPerSecond, maxLinearSpeedMetersPerSecond);
        omega = MathUtil.clamp(omega, -maxAngularSpeedRadPerSecond, maxAngularSpeedRadPerSecond);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                currentPose.getRotation()
        );

        return new AlignmentResult(branch, sectorIndex, targetPose, fieldRelativeSpeeds, robotRelativeSpeeds);
    }

    public record AlignmentResult(
            Branch branch,
            int sectorIndex,
            Pose2d targetPose,
            ChassisSpeeds fieldRelativeSpeeds,
            ChassisSpeeds robotRelativeSpeeds
    ) {}
}
