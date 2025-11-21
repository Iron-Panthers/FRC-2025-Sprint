package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
    Logger.recordOutput("Object Detection/Dy", getCoralDistanceY());
    Logger.recordOutput("Object Detection/Dx", getCoralDistanceX());
    Logger.recordOutput("Object Detection/Target Heading", getTargetRotation());
    Logger.recordOutput("Object Detection/Angle to target", getAngleToTarget());
    Logger.recordOutput("Object Detection/Target position", getTargetPosition());
    Logger.recordOutput("Object Detection/Detected coral", coralInVision());
  }

  /** whether or not we currently see a coral */
  public boolean coralInVision() {
    return inputs.targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0;
  }

  /**
   * Gets the target heading for the robot in order to make it go straight toward the object
   *
   * @return
   */
  public Rotation2d getTargetRotation() {
    if (!coralInVision()) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose().getRotation();
    }

    Angle angleToTarget = getAngleToTarget();

    return RobotState.getInstance()
        .getEstimatedPose()
        .getRotation()
        .minus(new Rotation2d(angleToTarget).plus(new Rotation2d(Math.PI)));
  }

  public Pose2d getTargetPosition() {
    if (!coralInVision()) {
      return RobotState.getInstance().getEstimatedPose();
    }
    Pose2d currentPose2d =
        RobotState.getInstance()
            .getEstimatedPose()
            .plus(
                new Transform2d(
                    new Translation2d(),
                    new Rotation2d(Units.Degrees.of(180)))); // the intake is on the back side

    // adjust for coral offset
    Pose2d finalPose =
        currentPose2d.plus(
            new Transform2d(
                new Translation2d(getCoralDistanceY(), getCoralDistanceX()), new Rotation2d()));

    finalPose = new Pose2d(finalPose.getTranslation(), getTargetRotation());

    return finalPose;
  }

  public Angle getAngleToTarget() {

    double distanceY = getCoralDistanceY().in(Units.Meters);
    double distanceX = getCoralDistanceX().in(Units.Meters);

    // arctan(t_x / t_y)
    Angle angleToTarget = Units.Radians.of(Math.atan2(distanceX, distanceY)); // radians
    return angleToTarget;
  }

  public Distance getCoralDistanceY() {
    Angle targetPitch = inputs.yErr;

    // t_y = c_z * tan(c_pitch + t_pitch) - c_y
    double distanceY =
        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                * Math.tan(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                        .pitchAngle()
                        .plus(targetPitch)
                        .in(Units.Radians))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters);
    return Units.Meters.of(distanceY);
  }

  public Distance getCoralDistanceX() {
    Angle targetPitch = inputs.yErr;
    Angle targetYaw = inputs.xErr;

    // t_x = (c_y + distanceY) * tan(t_yaw) + c_x
    double distanceX =
        (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters)
                    + getCoralDistanceY().in(Units.Meters))
                * Math.tan(
                    targetYaw
                        .plus(ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.yawAngle())
                        .in(Units.Radians))
            + ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.x().in(Units.Meters);
    return Units.Meters.of(distanceX);
  }
}
