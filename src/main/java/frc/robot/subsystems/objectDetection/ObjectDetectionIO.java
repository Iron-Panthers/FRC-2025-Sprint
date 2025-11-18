package frc.robot.subsystems.objectDetection;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public Angle xErr = Units.Degrees.of(0);
    public Angle yErr = Units.Degrees.of(0);
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
