package frc.robot.subsystems.intake.intake_sensors;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSensorsIO {
  @AutoLog
  class IntakeSensorsIOInputs {
    public boolean connected = false;
    public double distance = 0.0;
    public boolean isDetected = false;
  }

  default void updateInputs(IntakeSensorsIOInputs inputs) {}
}
