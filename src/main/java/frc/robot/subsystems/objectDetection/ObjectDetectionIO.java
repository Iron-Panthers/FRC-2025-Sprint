package frc.robot.subsystems.objectDetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public double xErr = 0;
    public double yErr = 0;
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
