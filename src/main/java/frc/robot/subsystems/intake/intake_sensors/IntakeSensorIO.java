package frc.robot.subsystems.intake.intake_sensors;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSensorIO {
  @AutoLog
  class IntakeSensorIOInputs {
    public boolean connected = false;
    public double distance = 0.0;
    public boolean isDetected = false;
  }

  default void updateInputs(IntakeSensorIOInputs inputs) {}
}
