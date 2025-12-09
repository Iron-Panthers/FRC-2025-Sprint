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
  private final ObjectDetectionIOInputsTwoAutoLogged inputsTwo;
  // make a velocity algorithm (maybe)

  public ObjectDetection(ObjectDetectionIO objectDetectionIO) {
    this.objectDetectionIO = objectDetectionIO;
    inputs = new ObjectDetectionIOInputsAutoLogged();
    inputsTwo = new ObjectDetectionIOInputsTwoAutoLogged();
  }

  @Override
  public void periodic() {
    // Camera one
    objectDetectionIO.updateInputs(inputs);
    Logger.processInputs("Object Detection", inputs);

    Logger.recordOutput("Object Detection/Error Horizontal", inputs.xErr);
    Logger.recordOutput("Object Detection/Error Vertical", inputs.yErr);
    Logger.recordOutput("Object Detection/Dy", getCoralDistanceY(1));
    Logger.recordOutput("Object Detection/Dx", getCoralDistanceX(1));
    Logger.recordOutput("Object Detection/Target Heading", getTargetRotation());
    Logger.recordOutput("Object Detection/Angle to target", getAngleToTarget(1));
    Logger.recordOutput("Object Detection/Target position", getTargetPosition());
    Logger.recordOutput("Object Detection/Detected coral", coralInVision(1));

    // Camera two
    objectDetectionIO.updateInputs(inputsTwo);
    Logger.processInputs("Object Detection", inputsTwo);

    Logger.recordOutput("Object Detection/Error Horizontal", inputsTwo.xErr);
    Logger.recordOutput("Object Detection/Error Vertical", inputsTwo.yErr);
    Logger.recordOutput("Object Detection/Dy", getCoralDistanceY(2));
    Logger.recordOutput("Object Detection/Dx", getCoralDistanceX(2));
    Logger.recordOutput("Object Detection/Target Heading", getTargetRotation());
    Logger.recordOutput("Object Detection/Angle to target", getAngleToTarget(2));
    Logger.recordOutput("Object Detection/Target position", getTargetPosition());
    Logger.recordOutput("Object Detection/Detected coral", coralInVision(2));
  }

  /** whether or not we currently see a coral */
  public boolean coralInVision(int cameraNum) {
    if (cameraNum == 1) {
      return inputs.targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0;
    }
    if (cameraNum == 2) {
      return inputsTwo.targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0;
    }
    return false;
  }

  public int whichCamera() {
    double dyOne = 0;
    double dxOne = 0;
    double dyTwo = 0;
    double dxTwo = 0;
    if (coralInVision(1)) {
      dyOne = getCoralDistanceY(1).in(Units.Meters);
      dxOne = getCoralDistanceX(1).in(Units.Meters);
    }
    if (coralInVision(2)) {
      dyTwo = getCoralDistanceY(2).in(Units.Meters);
      dxTwo = getCoralDistanceX(2).in(Units.Meters);
    }

    if (Math.sqrt(dxOne * dxOne + dyOne * dyOne) < Math.sqrt(dxTwo * dxTwo + dyTwo * dyTwo)) {
      return 1;
    }
    if (Math.sqrt(dxOne * dxOne + dyOne * dyOne) > Math.sqrt(dxTwo * dxTwo + dyTwo * dyTwo)) {
      return 2;
    }
    return -1;
  }

  public boolean bothCoralsInVision() {
    return (!coralInVision(1) && !coralInVision(2));
  }

  /**
   * Gets the target heading for the robot in order to make it go straight toward the object
   *
   * @return
   */
  public Rotation2d getTargetRotation() {
    if (!coralInVision(1)
        && !coralInVision(2)) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose().getRotation();
    }

    Angle angleToTarget = getAngleToTarget(whichCamera());

    return RobotState.getInstance()
        .getEstimatedPose()
        .getRotation()
        .minus(new Rotation2d(angleToTarget));
  }

  public Pose2d getTargetPosition() {
    if (!coralInVision(1)
        && !coralInVision(2)) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose();
    }

    int cameraNum = whichCamera();
    Translation2d translation =
        new Translation2d(getCoralDistanceY(cameraNum), getCoralDistanceX(cameraNum).times(-1));

    Transform2d targetPoseRelative = new Transform2d(translation, new Rotation2d());
    Logger.recordOutput("Object Detect/Target position relative", targetPoseRelative);

    Pose2d targetPose = RobotState.getInstance().getEstimatedPose().plus(targetPoseRelative);
    Logger.recordOutput("Object Detect/Target pose intermediary", targetPose);

    targetPose = new Pose2d(targetPose.getTranslation(), getTargetRotation());

    return targetPose;
  }

  public Angle getAngleToTarget(int cameraNum) {

    double distanceY = getCoralDistanceY(cameraNum).in(Units.Meters);
    double distanceX = getCoralDistanceX(cameraNum).in(Units.Meters);

    // arctan(t_x / t_y)
    Angle angleToTarget = Units.Radians.of(Math.atan2(distanceX, distanceY)); // radians
    return angleToTarget;
  }

  public Distance getCoralDistanceY(int cameraNum) {
    if (cameraNum == 1) {
      Angle targetPitch = inputs.yErr;
      Angle targetYaw = inputs.xErr;

      double distanceY =
          Math.cos(
                      ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                          .yawAngle()
                          .plus(targetYaw)
                          .in(Units.Radians))
                  * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                      * Math.tan(
                          ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                              .pitchAngle()
                              .plus(targetPitch)
                              .in(Units.Radians))
                      / Math.cos(targetYaw.in(Units.Radians)))
              - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters);
      return Units.Meters.of(distanceY);
    } else if (cameraNum == 2) {
      Angle targetPitch = inputs.yErr;
      Angle targetYaw = inputs.xErr;

      double distanceY =
          Math.cos(
                      ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO
                          .yawAngle()
                          .plus(targetYaw)
                          .in(Units.Radians))
                  * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO.z().in(Units.Meters)
                      * Math.tan(
                          ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO
                              .pitchAngle()
                              .plus(targetPitch)
                              .in(Units.Radians))
                      / Math.cos(targetYaw.in(Units.Radians)))
              - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO.y().in(Units.Meters);
      return Units.Meters.of(distanceY);
    }
    return null;
  }

  public Distance getCoralDistanceX(int cameraNum) {
    if (cameraNum == 1) {
      Angle targetPitch = inputs.yErr;
      Angle targetYaw = inputs.xErr;

      double distanceX =
          (Math.sin(
                      ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                          .yawAngle()
                          .plus(targetYaw)
                          .in(Units.Radians))
                  * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                      * Math.tan(
                          ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                              .pitchAngle()
                              .plus(targetPitch)
                              .in(Units.Radians))
                      / Math.cos(targetYaw.in(Units.Radians)))
              - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.x().in(Units.Meters));
      return Units.Meters.of(distanceX);
    } else if (cameraNum == 2) {
      Angle targetPitch = inputs.yErr;
      Angle targetYaw = inputs.xErr;

      double distanceX =
          (Math.sin(
                      ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO
                          .yawAngle()
                          .plus(targetYaw)
                          .in(Units.Radians))
                  * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO.z().in(Units.Meters)
                      * Math.tan(
                          ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO
                              .pitchAngle()
                              .plus(targetPitch)
                              .in(Units.Radians))
                      / Math.cos(targetYaw.in(Units.Radians)))
              - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS_TWO.x().in(Units.Meters));
      return Units.Meters.of(distanceX);
    }
    return null;
  }
}
