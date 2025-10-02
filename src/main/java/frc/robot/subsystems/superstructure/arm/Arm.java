package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Arm extends GenericSuperstructure<Arm.ArmTarget> implements LoggableMechanism3d {
  public enum ArmTarget implements GenericSuperstructure.PositionTarget {
    TOP(-79), // TODO: need to tune these values
    INTAKE(-96),
    STOW(-96);

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
    setPositionTarget(ArmTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  /**
   * The parent LoggableMechanism3d, typically a reference to the elevator
   * subsystem
   */
  public LoggableMechanism3d loggableMechanism3dParent = null;

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        "Superstructure/Arm/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
  }

  /**
   * This function returns whether or not the subsystem has reached its position
   * target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d)) <= super.getPositionTarget()
        .getEpsilon();
  }

  /**
   * Returns the position of the arm in DEGREES
   */
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
