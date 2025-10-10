package frc.robot.subsystems.intake.intakeSensors;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSensorsIO {
  @AutoLog
  // this is public static anyways since all members of an interface
  // are public and any nested type is automatically static
  class IntakeSensorsIOInputs { //basically default values that will get updated
    public boolean connected = false;
    public double distance = 0.0;
    public boolean isDetected = false;
  }

  default void updateInputs(IntakeSensorsIOInputs inputs) {}
}
