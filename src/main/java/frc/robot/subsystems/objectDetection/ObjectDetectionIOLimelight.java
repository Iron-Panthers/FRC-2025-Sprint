package frc.robot.subsystems.objectDetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public ObjectDetectionIOLimelight() {}

  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.xErr = table.getEntry("tx").getDouble(0);
    inputs.yErr = table.getEntry("ty").getDouble(0);
  }
}
