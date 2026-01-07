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
  private ObjectDetectionIO[] objectDetectionIOs;
  private final ObjectDetectionIOInputsAutoLogged[] inputs;
  // make a velocity algorithm (maybe)

  public ObjectDetection(ObjectDetectionIO... objectDetectionIOs) {
    this.objectDetectionIOs = objectDetectionIOs;
    inputs = ObjectDetectionIOInputsAutoLogged[objectDetectionIOs.length]; //TODO: MAybe oingaerigiweofnaweopifkndpesnt work
    for(int i = 0; i < objectDetectionIOs.length; i++){
      inputs[i] = new ObjectDetectionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Camera one
    objectDetectionIO.updateInputs(inputs);
    Logger.processInputs("Object Detection", inputs);

    Logger.recordOutput("Object Detection/Error Horizontal", inputs.xErr);
    Logger.recordOutput("Object Detection/Error Vertical", inputs.yErr);
    Logger.recordOutput("Object Detection/Dy", getCoralDistanceY(i));
    Logger.recordOutput("Object Detection/Dx", getCoralDistanceX(i));
    Logger.recordOutput("Object Detection/Target Heading", getTargetRotation());
    Logger.recordOutput("Object Detection/Angle to target", getAngleToTarget());
    Logger.recordOutput("Object Detection/Target position", getTargetPosition());
    Logger.recordOutput("Object Detection/Detected coral", coralInVision());
  }

  /** whether or not we currently see a coral */
  public boolean coralInVision() {
    return inputs[0].targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0 
    || inputs[1].targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0;
  }

  public int whichCamera() {
    double dyOne = 0;
    double dxOne = 0;
    double dyTwo = 0;
    double dxTwo = 0;
    if (coralInVision()) {
      dyOne = getCoralDistanceY(i).in(Units.Meters);
      dxOne = getCoralDistanceX(i).in(Units.Meters);
    }
    ;

    if (Math.sqrt(dxOne * dxOne + dyOne * dyOne) < Math.sqrt(dxTwo * dxTwo + dyTwo * dyTwo)) {
      return 1;
    }
    if (Math.sqrt(dxOne * dxOne + dyOne * dyOne) > Math.sqrt(dxTwo * dxTwo + dyTwo * dyTwo)) {
      return 2;
    }
    return -1;
  }

  public boolean bothCoralsInVision() {
    return (!coralInVision());
  }

  /**
   * Gets the target heading for the robot in order to make it go straight toward the object
   *
   * @return
   */
  public Rotation2d getTargetRotation() {
    if (!coralInVision()
        && !coralInVision()) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose().getRotation();
    }

    Angle angleToTarget = getAngleToTarget();

    return RobotState.getInstance()
        .getEstimatedPose()
        .getRotation()
        .minus(new Rotation2d(angleToTarget));
  }

  public Pose2d getTargetPosition() {
    if (!coralInVision()
        && !coralInVision()) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose();
    }

    Translation2d translation =
        new Translation2d(getCoralDistanceY(i), getCoralDistanceX(i).times(-1));

    Transform2d targetPoseRelative = new Transform2d(translation, new Rotation2d());
    Logger.recordOutput("Object Detect/Target position relative", targetPoseRelative);

    Pose2d targetPose = RobotState.getInstance().getEstimatedPose().plus(targetPoseRelative);
    Logger.recordOutput("Object Detect/Target pose intermediary", targetPose);

    targetPose = new Pose2d(targetPose.getTranslation(), getTargetRotation());

    return targetPose;
  }

  public Angle getAngleToTarget() {

    double distanceY = getCoralDistanceY(i).in(Units.Meters);
    double distanceX = getCoralDistanceX(i).in(Units.Meters);

    // arctan(t_x / t_y)
    Angle angleToTarget = Units.Radians.of(Math.atan2(distanceX, distanceY)); // radians
    return angleToTarget;
  }

  // Switch between cameras
  public int findCameraIndex(){
    if (getCoralDistanceY(0).magnitude() < getCoralDistanceY(1).magnitude()){
      return 0;
    }
    return 1;
  }

  public Distance getCoralDistanceY(int i) {
    Angle targetPitch = inputs[i].yErr;
    Angle targetYaw = inputs[i].xErr;

    double distanceY =
        Math.cos(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i]
                        .yawAngle()
                        .plus(targetYaw)
                        .in(Units.Radians))
                * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i].z().in(Units.Meters)
                    * Math.tan(
                        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i]
                            .pitchAngle()
                            .plus(targetPitch)
                            .in(Units.Radians))
                    / Math.cos(targetYaw.in(Units.Radians)))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i].y().in(Units.Meters);
    return Units.Meters.of(distanceY);
  }

  public Distance getCoralDistanceX(int i) {
    Angle targetPitch = inputs[i].yErr;
    Angle targetYaw = inputs[i].xErr;

    double distanceX =
        (Math.sin(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i]
                        .yawAngle()
                        .plus(targetYaw)
                        .in(Units.Radians))
                * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i].z().in(Units.Meters)
                    * Math.tan(
                        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i]
                            .pitchAngle()
                            .plus(targetPitch)
                            .in(Units.Radians))
                    / Math.cos(targetYaw.in(Units.Radians)))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS[i].x().in(Units.Meters));
    return Units.Meters.of(distanceX);
  }
}
