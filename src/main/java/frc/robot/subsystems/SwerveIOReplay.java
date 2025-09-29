package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Replays swerve inputs from a previously recorded WPILib data log. */
public class SwerveIOReplay implements SwerveIO {
    private final List<SwerveIOInputs> samples = new ArrayList<>();
    private final List<Double> sampleTimes = new ArrayList<>();

    private boolean started = false;
    private double startTimestamp = 0.0;
    private int currentSample = 0;

    public SwerveIOReplay(Path logPath) {
        if (!Files.exists(logPath)) {
            throw new IllegalArgumentException("Replay log does not exist: " + logPath);
        }

        loadSamples(logPath);
    }

    private void loadSamples(Path logPath) {
        Map<Integer, String> entryNames = new HashMap<>();
        List<double[]> moduleStatesData = new ArrayList<>();
        List<double[]> modulePositionsData = new ArrayList<>();
        List<double[]> gyroData = new ArrayList<>();
        List<Double> timestamps = new ArrayList<>();

        DataLogReader reader;
        try {
            reader = new DataLogReader(logPath.toString());
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to read replay log", e);
        }

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                entryNames.put(record.getStartData().entry, record.getStartData().name);
            } else if (record.isControl()) {
                continue;
            } else {
                String name = entryNames.get(record.getEntry());
                if (name == null) {
                    continue;
                }

                if (SwerveIOInputs.MODULE_STATE_LOG_ENTRY.equals(name)) {
                    moduleStatesData.add(record.getDoubleArray());
                    timestamps.add(record.getTimestamp() * 1.0e-6);
                } else if (SwerveIOInputs.MODULE_POSITION_LOG_ENTRY.equals(name)) {
                    modulePositionsData.add(record.getDoubleArray());
                } else if (SwerveIOInputs.GYRO_LOG_ENTRY.equals(name)) {
                    gyroData.add(record.getDoubleArray());
                }
            }
        }

        int sampleCount = Math.min(moduleStatesData.size(), Math.min(modulePositionsData.size(), gyroData.size()));
        if (sampleCount == 0) {
            throw new IllegalStateException("Replay log did not contain any swerve telemetry");
        }

        double baseTimestamp = timestamps.isEmpty() ? 0.0 : timestamps.get(0);

        for (int i = 0; i < sampleCount; i++) {
            double[] stateArray = moduleStatesData.get(i);
            double[] positionArray = modulePositionsData.get(i);
            double[] gyroArray = gyroData.get(i);

            SwerveIOInputs sample = new SwerveIOInputs();
            SwerveModuleState[] states = SwerveIOInputs.expandModuleStates(stateArray);
            SwerveModulePosition[] positions = SwerveIOInputs.expandModulePositions(positionArray);
            sample.setModuleStates(states);
            sample.setModulePositions(positions);
            sample.gyroYaw = gyroArray.length > 0 ? new Rotation2d(gyroArray[0]) : new Rotation2d();
            sample.gyroPitch = gyroArray.length > 1 ? new Rotation2d(gyroArray[1]) : new Rotation2d();
            sample.gyroRoll = gyroArray.length > 2 ? new Rotation2d(gyroArray[2]) : new Rotation2d();
            sample.chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(states);

            samples.add(sample);
            double timestampSeconds = timestamps.size() > i ? timestamps.get(i) - baseTimestamp : i * 0.02;
            sampleTimes.add(Math.max(0.0, timestampSeconds));
        }
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        if (samples.isEmpty()) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (!started) {
            started = true;
            startTimestamp = now;
        }

        double elapsed = now - startTimestamp;
        while (currentSample < samples.size() - 1 && sampleTimes.get(currentSample + 1) <= elapsed) {
            currentSample++;
        }

        inputs.copyFrom(samples.get(currentSample));
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, ChassisSpeeds referenceSpeeds, boolean isOpenLoop) {
        // Replay data ignores outgoing commands.
    }

    @Override
    public void resetModulesToAbsolute() {
        currentSample = 0;
        started = false;
    }

    @Override
    public void zeroHeading() {
        currentSample = 0;
        started = false;
    }

    @Override
    public void stop() {}
}
