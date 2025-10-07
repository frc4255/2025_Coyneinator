import pathlib
path = pathlib.Path('src/main/java/frc/robot/subsystems/Vision/VisionSubsystem.java')
text = path.read_text()
path.write_text(text)
