package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.subsystems.superstructure.SuperstructureController.ArmDirection;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Arm extends GenericSuperstructure<Arm.ArmTarget> implements LoggableMechanism3d {
  public enum ArmTarget implements GenericSuperstructure.PositionTarget {
    TOP(90),
    PICKUP(-90),
    STRAIGHT(0),
    LEFT(180);

    private double position;
    private static final double EPSILON = ArmConstants.POSITION_TARGET_EPSILON;

    private ArmTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }
  }

  public Arm(ArmIO io) {
    super("Arm", io);
    setPositionTarget(ArmTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  /** The parent LoggableMechanism3d, typically a reference to the elevator subsystem */
  public LoggableMechanism3d loggableMechanism3dParent = null;

  /**
   * The target angle of the arm relative to the horizontal plane, changed later based on the
   * current position
   */
  private Angle absoluteTargetAngleManual;

  public void setAbsoluteTargetAngleManual(Angle angle) {
    this.absoluteTargetAngleManual = angle;
  }

  public Angle getAbsoluteTargetAngleManual() {
    return absoluteTargetAngleManual;
  }

  private ArmDirection armDirection = ArmDirection.BOTH;

  public void setArmDirection(ArmDirection direction) {
    this.armDirection = direction;
  }

  public ArmDirection getArmDirection() {
    return armDirection;
  }

  /**
   * Gets the relative angle to give to the motor controller to reach the given absolute angle
   *
   * @param absoluteAngle (0-360 degrees) centered at directly right when looking from the intake
   *     side
   * @return the relative angle to give to the motor controller
   */
  public double absoluteToRelativeTarget(double absoluteAngle) {
    double currentAngle = getPosition();
    double absolutePosition = currentAngle % 360.0;
    double deltaAngle = normalizeAngle(absoluteAngle - absolutePosition);
    double clockwise = absolutePosition + deltaAngle;
    double counterClockwise = absolutePosition + deltaAngle - 360.0;

    double finalTarget =
        switch (armDirection) {
          case CLOCKWISE -> clockwise;
          case COUNTERCLOCKWISE -> counterClockwise;
          case BOTH -> (Math.abs(deltaAngle) < Math.abs(deltaAngle - 360.0))
              ? clockwise
              : counterClockwise;
        };
    return finalTarget;
  }

  /**
   * Normalizes between -360 and 360
   *
   * @param angle
   * @return
   */
  public double normalizeAngle(double angle) {
    double sign = Math.signum(angle);
    angle = Math.abs(angle);
    angle = angle % 360.0;
    return angle * sign;
  }

  @Override
  public void periodic() {
    double relativeAngle = absoluteToRelativeTarget(absoluteTargetAngleManual.in(Units.Degrees));
    super.setPositionTargetManual(relativeAngle);

    super.periodic();

    Logger.recordOutput(
        "Superstructure/Arm/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
        <= super.getPositionTarget().getEpsilon();
  }

  /** Returns the position of the arm in DEGREES */
  public double getPosition() {
    return super.getPosition() * 360.0;
  }

  // ----- LoggableMechanism3d methods
  @Override
  public Pose3d getParentPosition() {
    if (loggableMechanism3dParent != null) {
      return loggableMechanism3dParent.getDisplayPose3d();
    }
    return new Pose3d();
  }

  @Override
  public void setParent(LoggableMechanism3d parent) {
    if (parent == null) {
      throw new IllegalArgumentException("Parent cannot be null");
    }
    if (parent == this) {
      throw new IllegalArgumentException("Parent cannot be itself");
    }
    this.loggableMechanism3dParent = parent;
  }

  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(ArmConstants.ELEVATOR_TO_ARM_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, Math.toRadians(getPosition() - 90), 0)));
  }
}
