package frc.robot.subsystems.l1_pivot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class L1Pivot extends GenericSuperstructure<L1Pivot.L1PivotTarget>
    implements LoggableMechanism3d {
  public enum L1PivotTarget implements GenericSuperstructure.PositionTarget {
    L1_SCORE(116.28),
    STOW(140),
    CLIMB(0);

    private double position;
    private static final double EPSILON = L1PivotConstants.POSITION_TARGET_EPSILON;

    private L1PivotTarget(double position) {
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

  private final LinearFilter supplyCurrentFilter;

  private LoggableMechanism3d loggableMechanism3dParent = null;

  private double filteredSupplyCurrentAmps = 0;

  private boolean zeroing = false;

  public L1Pivot(L1PivotIO io) {
    super("L1Pivot", io);
    setPositionTarget(L1PivotTarget.STOW);
    setControlMode(ControlMode.STOP);
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {
    super.periodic();

    // for zeroing
    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());

    // run characterization if we are zeroing
    if (zeroing) {
      superstructureIO.runCharacterization();
    }

    // record our outputs
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
    Logger.recordOutput(
        "Superstructure/L1Pivot/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
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

  // TODO: Convert from double to WPILib units! (every where else too)
  public double getPosition() {
    return super.getPosition() * 360.0;
  }

  /**
   * Returns weather or not the subsystems is zeroing
   *
   * @return whether or not the subsystem is zeroing
   */
  public boolean getZeroing() {
    return zeroing;
  }

  /**
   * Sets weather or not the subsystem should be zeroing
   *
   * @param zeroing
   */
  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  @Override
  public Pose3d getParentPosition() {
    if (loggableMechanism3dParent != null) {
      return loggableMechanism3dParent.getDisplayPose3d();
    }
    return new Pose3d();
  }

  /** Gets the filtered supply current amps for the subsystem Used for zeroing */
  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
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
        .plus(L1PivotConstants.ELEVATOR_TO_L1_PIVOT_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, -Math.toRadians(getPosition() + 90), 0)));
  }
}
