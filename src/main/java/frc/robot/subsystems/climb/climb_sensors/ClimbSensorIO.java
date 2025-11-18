package frc.robot.subsystems.climb.climb_sensors;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbSensorIO {
  @AutoLog
  class ClimbSensorIOInputs {
    public boolean connected = false;
    public boolean triggered = false;
  }

  default void updateInputs(ClimbSensorIOInputs inputs) {}
}
