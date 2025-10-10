package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AutoAlignHelper;

/**
 * Auto align using the original Choreo-based trajectory follower.
 */
public class AutoAlignToClosestBranch extends Command {
    private static final double MAX_TRANSLATION_SPEED = Math.min(2.5, Constants.Swerve.maxSpeed);
    private static final double MAX_TRANSLATION_ACCEL = 2.5;
    private static final double POSITION_TOLERANCE_METERS = 0.04;
    private static final double ROTATION_TOLERANCE_RAD = Math.toRadians(4.0);
    private static final double PROGRESS_DROP_REPLAN_METERS = 0.05;
    private static final double MIN_TIME_BEFORE_REPLAN_S = 0.2;
    private static final double REEF_CLEARANCE_METERS = Units.inchesToMeters(0.125);
    private static final double ROBOT_RADIUS_METERS =
        Math.hypot(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
    private static final double KEEP_OUT_RADIUS =
        FieldLayout.Reef.reefHitboxRadius + ROBOT_RADIUS_METERS + REEF_CLEARANCE_METERS;
    private static final double AVOID_BUFFER_METERS = Units.inchesToMeters(1.0);

    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final DoubleSupplier translationInput;
    private final DoubleSupplier strafeInput;
    private final DoubleSupplier rotationInput;

    private Pose2d goalPose = new Pose2d();
    private char lockedBranch = 'A';
    private Trajectory<SwerveSample> trajectory;
    private double totalTimeSeconds;
    private double pathLengthMeters;
    private double bestProgressMeters;
    private boolean motionComplete;
    private boolean driverCancel;

    public AutoAlignToClosestBranch(
        Swerve swerve,
        DoubleSupplier translationInput,
        DoubleSupplier strafeInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.translationInput = translationInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d startPose = swerve.getPose();
        driverCancel = false;
        motionComplete = false;

        lockedBranch = determineClosestBranch();
        goalPose = AutoAlignHelper.computeBranchPose(lockedBranch);
        logTarget();

        buildTrajectory(startPose, goalPose);

        bestProgressMeters = 0.0;
        if (trajectory == null) {
            motionComplete = true;
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        logNearestBranch();

        if (motionComplete) {
            holdPosition();
            return;
        }

        if (hasDriverOverride()) {
            driverCancel = true;
            holdPosition();
            return;
        }

        if (trajectory == null) {
            motionComplete = true;
            holdPosition();
            return;
        }

        double elapsed = Math.min(timer.get(), totalTimeSeconds);
        Optional<SwerveSample> sample = trajectory.sampleAt(elapsed, false);
        if (sample.isPresent()) {
            swerve.followTrajectory(sample.get());
        } else {
            holdPosition();
        }

        updateProgress(currentPose);
    }

    @Override
    public boolean isFinished() {
        return motionComplete || driverCancel;
    }

    @Override
    public void end(boolean interrupted) {
        holdPosition();
        if (!interrupted && motionComplete && !driverCancel) {
            swerve.setHeading(goalPose.getRotation());
        }

        if (driverCancel) {
            lockedBranch = determineClosestBranch();
            goalPose = AutoAlignHelper.computeBranchPose(lockedBranch);
            logTarget();
        }
    }

    private void updateProgress(Pose2d currentPose) {
        double distanceRemaining = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        double distanceCovered = Math.max(0.0, pathLengthMeters - distanceRemaining);
        distanceCovered = Math.min(distanceCovered, pathLengthMeters);

        if (distanceCovered > bestProgressMeters + 0.01) {
            bestProgressMeters = distanceCovered;
        } else if (
            distanceCovered < bestProgressMeters - PROGRESS_DROP_REPLAN_METERS
                && timer.get() > MIN_TIME_BEFORE_REPLAN_S
        ) {
            rebuildTrajectory(currentPose);
            return;
        }

        boolean translationSettled = distanceRemaining < POSITION_TOLERANCE_METERS;
        boolean rotationSettled = Math.abs(
            currentPose.getRotation().minus(goalPose.getRotation()).getRadians()
        ) < ROTATION_TOLERANCE_RAD;

        if (timer.get() >= totalTimeSeconds && translationSettled && rotationSettled) {
            motionComplete = true;
        }
    }

    private void rebuildTrajectory(Pose2d currentPose) {
        buildTrajectory(currentPose, goalPose);
        bestProgressMeters = 0.0;
        timer.reset();
        timer.start();
    }

    private void holdPosition() {
        swerve.drive(new Translation2d(), 0.0, true, false);
    }

    private char determineClosestBranch() {
        return swerve.findClosestBranch(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        );
    }

    private boolean hasDriverOverride() {
        return Math.abs(translationInput.getAsDouble()) > Constants.stickDeadband
            || Math.abs(strafeInput.getAsDouble()) > Constants.stickDeadband
            || Math.abs(rotationInput.getAsDouble()) > Constants.stickDeadband;
    }

    private void buildTrajectory(Pose2d startPose, Pose2d endPose) {
        List<Translation2d> points = buildPathPoints(startPose.getTranslation(), endPose.getTranslation());
        pathLengthMeters = computeTotalLength(points);

        if (pathLengthMeters < POSITION_TOLERANCE_METERS && Math.abs(
            startPose.getRotation().minus(endPose.getRotation()).getRadians()
        ) < ROTATION_TOLERANCE_RAD) {
            trajectory = null;
            totalTimeSeconds = 0.0;
            motionComplete = true;
            return;
        }

        double desiredSpeed = Math.min(MAX_TRANSLATION_SPEED, Constants.Swerve.maxSpeed);
        totalTimeSeconds = Math.max(pathLengthMeters / Math.max(desiredSpeed, 1e-6), 0.25);

        Rotation2d startHeading = startPose.getRotation();
        double deltaHeading = MathUtil.angleModulus(
            endPose.getRotation().minus(startHeading).getRadians()
        );

        List<SwerveSample> samples = new ArrayList<>();
        int sampleCount = Math.max(2, (int) Math.ceil(totalTimeSeconds / 0.02));
        double timeStep = totalTimeSeconds / (sampleCount - 1);

        double accumulated = 0.0;
        int segmentIndex = 0;
        Translation2d segmentStart = points.get(0);
        Translation2d segmentEnd = points.get(1);
        double segmentLength = segmentStart.getDistance(segmentEnd);

        for (int i = 0; i < sampleCount; i++) {
            double time = i * timeStep;
            double progressMeters = Math.min(
                pathLengthMeters,
                (time / totalTimeSeconds) * pathLengthMeters
            );

            while (
                segmentIndex < points.size() - 2
                    && progressMeters > accumulated + segmentLength - 1e-6
            ) {
                accumulated += segmentLength;
                segmentIndex++;
                segmentStart = points.get(segmentIndex);
                segmentEnd = points.get(segmentIndex + 1);
                segmentLength = segmentStart.getDistance(segmentEnd);
            }

            double segmentProgress = segmentLength < 1e-6
                ? 0.0
                : (progressMeters - accumulated) / segmentLength;
            segmentProgress = MathUtil.clamp(segmentProgress, 0.0, 1.0);

            Translation2d position = segmentStart.plus(
                segmentEnd.minus(segmentStart).times(segmentProgress)
            );

            Translation2d direction = segmentEnd.minus(segmentStart);
            double length = direction.getNorm();
            if (length > 1e-6) {
                direction = direction.div(length);
            } else {
                direction = new Translation2d();
            }

            double heading = startHeading.getRadians() + deltaHeading * (time / totalTimeSeconds);
            double vx = desiredSpeed * direction.getX();
            double vy = desiredSpeed * direction.getY();
            double omega = totalTimeSeconds > 1e-6 ? deltaHeading / totalTimeSeconds : 0.0;

            if (i == sampleCount - 1) {
                vx = 0.0;
                vy = 0.0;
                omega = 0.0;
            }

            samples.add(new SwerveSample(
                time,
                position.getX(),
                position.getY(),
                heading,
                vx,
                vy,
                omega,
                0.0,
                0.0,
                0.0,
                new double[4],
                new double[4]
            ));
        }

        trajectory = new Trajectory<>(
            "AutoAlignDynamic",
            samples,
            List.of(),
            List.of()
        );
    }

    private List<Translation2d> buildPathPoints(Translation2d start, Translation2d end) {
        List<Translation2d> points = new ArrayList<>();
        points.add(start);
        points.add(end);

        boolean changed;
        do {
            changed = false;
            for (Translation2d center : reefCenters()) {
                for (int i = 0; i < points.size() - 1; i++) {
                    Translation2d a = points.get(i);
                    Translation2d b = points.get(i + 1);
                    if (segmentIntersectsCircle(a, b, center, KEEP_OUT_RADIUS)) {
                        Translation2d avoid = chooseAvoidPoint(a, b, center, KEEP_OUT_RADIUS + AVOID_BUFFER_METERS);
                        points.add(i + 1, avoid);
                        changed = true;
                        break;
                    }
                }
                if (changed) {
                    break;
                }
            }
        } while (changed);

        return points;
    }

    private double computeTotalLength(List<Translation2d> points) {
        double length = 0.0;
        for (int i = 0; i < points.size() - 1; i++) {
            length += points.get(i).getDistance(points.get(i + 1));
        }
        return length;
    }

    private double computeStartVelocity(Pose2d startPose, Pose2d endPose) {
        Translation2d delta = endPose.getTranslation().minus(startPose.getTranslation());
        double distance = delta.getNorm();
        if (distance < 1e-6) {
            return 0.0;
        }

        Translation2d direction = delta.div(distance);
        ChassisSpeeds robotSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds,
            startPose.getRotation()
        );

        Translation2d fieldVelocity = new Translation2d(
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond
        );

        double projection = fieldVelocity.getX() * direction.getX()
            + fieldVelocity.getY() * direction.getY();

        double startVelocity = Math.max(0.0, projection);
        return MathUtil.clamp(startVelocity, 0.0, MAX_TRANSLATION_SPEED);
    }

    private List<Translation2d> reefCenters() {
        return List.of(
            FieldLayout.Reef.center,
            new Translation2d(
                FieldLayout.FIELD_LENGTH - FieldLayout.Reef.center.getX(),
                FieldLayout.Reef.center.getY()
            )
        );
    }

    private boolean segmentIntersectsCircle(
        Translation2d start,
        Translation2d end,
        Translation2d center,
        double radius
    ) {
        double radiusSq = radius * radius;
        boolean startInside = start.getDistance(center) <= radius;
        boolean endInside = end.getDistance(center) <= radius;

        if (startInside || endInside) {
            return false;
        }

        Translation2d delta = end.minus(start);
        double lengthSq = delta.getX() * delta.getX() + delta.getY() * delta.getY();
        if (lengthSq < 1e-6) {
            return start.getDistance(center) <= radius;
        }

        double t = ((center.getX() - start.getX()) * delta.getX()
            + (center.getY() - start.getY()) * delta.getY()) / lengthSq;
        t = MathUtil.clamp(t, 0.0, 1.0);

        Translation2d projection = start.plus(delta.times(t));
        return projection.getDistance(center) <= radius;
    }

    private Translation2d chooseAvoidPoint(
        Translation2d start,
        Translation2d end,
        Translation2d center,
        double radius
    ) {
        Translation2d startDir = normalize(start.minus(center));
        Translation2d endDir = normalize(end.minus(center));
        Translation2d bisector = startDir.plus(endDir);

        if (bisector.getNorm() < 1e-6) {
            bisector = new Translation2d(-startDir.getY(), startDir.getX());
        }
        if (bisector.getNorm() < 1e-6) {
            bisector = new Translation2d(1.0, 0.0);
        }

        bisector = normalize(bisector);
        return center.plus(bisector.times(radius));
    }

    private Translation2d normalize(Translation2d vector) {
        double norm = vector.getNorm();
        if (norm < 1e-6) {
            return new Translation2d(0.0, 0.0);
        }
        return vector.div(norm);
    }

    private void logTarget() {
        Logger.recordOutput("AutoAlign/TargetBranch", String.valueOf(lockedBranch));
        Logger.recordOutput("AutoAlign/TargetPose", goalPose);
    }

    private void logNearestBranch() {
        Logger.recordOutput(
            "AutoAlign/NearestBranch",
            String.valueOf(determineClosestBranch())
        );
    }
}
