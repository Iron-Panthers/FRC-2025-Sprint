package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

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

    Logger.recordOutput("Object Detection/Error Horizontal", inputs.xErr);
    Logger.recordOutput("Object Detection/Error Vertical", inputs.yErr);
  }

  /**
   * Gets the target heading for the robot in order to make it go straight toward the object
   *
   * @return
   */
  public Rotation2d getTargetRotation() {
    Angle targetPitch = inputs.yErr;
    Angle targetYaw = inputs.xErr;

    // t_y = c_z * tan(c_pitch + t_pitch) - c_y
    double distanceY =
        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                * Math.tan(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                        .pitchAngle()
                        .plus(targetPitch)
                        .in(Units.Radians))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters);

    // t_x = (c_y + distanceY) * tan(t_yaw) + c_x
    double distanceX =
        (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters) + distanceY)
                * Math.tan(targetYaw.in(Units.Radians))
            + ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.x().in(Units.Meters);
    
    // arctan(t_x / t_y)
    double angleToTarget = Math.atan2(distanceX, distanceY); // radians

    Logger.recordOutput("ObjectDetection/Angle to Target", angleToTarget);

    return RobotState.getInstance()
        .getEstimatedPose()
        .getRotation()
        .minus(new Rotation2d(angleToTarget).plus(new Rotation2d(Math.PI)));
  }
  // Pass in a value so that the drive class rotates that many values; purpose is so that when a
  // button is pressed, the robot automatically aligns to the coral
  // In the drive class, use target position; change the angles in target position (gotta create a
  // method that changes it)
}
