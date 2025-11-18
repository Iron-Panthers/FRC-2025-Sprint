package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase {
  private ObjectDetectionIO objectDetectionIO;
  private final ObjectDetectionIOInputsAutoLogged inputs;
  // make a velocity algorithm (maybe)

  public ObjectDetection(ObjectDetectionIO objectDetectionIO) {
    this.objectDetectionIO = objectDetectionIO;
    inputs = new ObjectDetectionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    objectDetectionIO.updateInputs(inputs);
  }

  public Rotation2d getRotation() {
    return new Rotation2d(inputs.xErr);
  }
  // Pass in a value so that the drive class rotates that many values; purpose is so that when a
  // button is pressed, the robot automatically aligns to the coral
  // In the drive class, use target position; change the angles in target position (gotta create a
  // method that changes it)
}
