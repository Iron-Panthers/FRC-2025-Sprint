package frc.robot.subsystems.objectDetection;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ObjectDetectionConstants {

  public static final CameraPositionConstants CAMERA_POSITION_CONSTANTS =
      switch (Constants.getRobotType()) {
        default -> new CameraPositionConstants(
            Units.Meters.of(0.0), // x position
            Units.Meters.of(0.0), // y position
            Units.Meters.of(0.0), // z position
            Units.Degrees.of(0.0), // pitch angle
            Units.Degrees.of(0.0) // yaw angle
            );
      };

  /**
   * Camera position constants used for distance calculations
   *
   * @param x The x position of the camera from the center of the robot (left and right)
   * @param y The y position of the camera from the center of the robot (forward and backward)
   * @param z The z position of the camera from the center of the robot (up and down)
   */
  public static record CameraPositionConstants(
      Distance x, Distance y, Distance z, Angle pitchAngle, Angle yawAngle) {}
}
