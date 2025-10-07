# Coyneinator 2025 Robot Code

## Simulation quick start

You can drive the full robot codebase in WPILib sim with the same commands that run on the real bot.

```bash
bash ./gradlew simulateJava
```

By default the drivetrain will use the physics-backed `SwerveIOSim`, so joystick input is applied live inside the simulator. Use the `SWERVE_IO_MODE` (or `-DswerveIOMode=`) flag to pick a backend explicitly when testing:

| Mode value | Behavior |
| --- | --- |
| `SIM` | Force the live physics simulation even if a replay log is configured. |
| `REPLAY` | Play back a previously recorded drive by also setting `SWERVE_REPLAY_LOG` (or `-DswerveReplayLog=`) to the WPILib log file path. |
| `REAL` | Instantiate the TalonFX implementation in sim (useful for integration testing with CTRE’s networked simulator). |
| `AUTO` *(default)* | Use replay when a valid log is supplied, otherwise fall back to the live simulation. |

Example forcing live sim while keeping other defaults:

```bash
SWERVE_IO_MODE=SIM bash ./gradlew simulateJava
```

To replay a log file instead, run:

```bash
SWERVE_IO_MODE=REPLAY \
SWERVE_REPLAY_LOG=/path/to/run.wpilog \
bash ./gradlew simulateJava
```
