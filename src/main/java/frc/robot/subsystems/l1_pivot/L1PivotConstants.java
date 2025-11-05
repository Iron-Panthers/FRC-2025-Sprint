package frc.robot.subsystems.l1_pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;
import frc.robot.subsystems.l1_pivot.L1PivotConstants.L1PivotConfig;

// TODO: Change ALLLLL of these constants, these were just copied and pasted from Sim-2025
public class L1PivotConstants {
  public static final L1PivotConfig L1_PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new L1PivotConfig(CAN.at(33, "L1 Pivot"), 33.75);
        case SIM -> new L1PivotConfig(CAN.at(8, "L1 Pivot"), 3.75);
        default -> new L1PivotConfig(0, 3.75);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(2, 0, 0, 0, 3.846, 0.0769, 0.2);
        case SIM -> new PIDGains(5, 0, 0, 0, 0.47, 0.02, 0.26);
        default -> new PIDGains(10, 0, 0, 0, 0.47, 0.02, 0.26);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new MotionMagicConfig(6, 10);
        case COMP -> new MotionMagicConfig(7.5, 10);
        default -> new MotionMagicConfig(7.5, 10);
      };

  public record L1PivotConfig(int motorID, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = .05;
  public static final double L1_PIVOT_LENGTH = 13.138; // inches

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 6;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = 1;
  public static final double ZEROING_OFFSET = .2019; // offset in rotations
  public static final double ZEROING_VOLTAGE_THRESHOLD = 3.5;

  // TIMING CONSTANTS
  public static final double L1_SCORE_TIME_OFFSET = 0.1;

  // L1 PIVOT POSITION CONSTANTS
  public static final Transform3d ELEVATOR_TO_L1_PIVOT_TRANSFORM =
      switch (Constants.getRobotType()) {
        default -> new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.5), Units.inchesToMeters(0d), Units.inchesToMeters(33.875)),
            new Rotation3d(0, 0, 0));
      };

  // PHYSICAL CONSTANTS
  public static record L1PivotPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  public static final L1PivotPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new L1PivotPhysicalConstants(0.0109810104, 0.332194, -1000.0, 1000, false);
        case COMP -> new L1PivotPhysicalConstants(0.0109810104, 0.332194, 0, 0, false);
        default -> new L1PivotPhysicalConstants(0.0109810104, 0.332194, 0, 0, false);
      };
}
