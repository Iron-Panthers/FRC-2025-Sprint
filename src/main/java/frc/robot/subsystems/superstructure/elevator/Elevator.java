package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget>
    implements LoggableMechanism3d {

  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0.6),
    L1(11),
    L2(0),
    L3(5),
    L4(32.5),
    ALGAE_SCORE_PROCESSOR(5),
    ALGAE_SCORE_NET(30),
    TOP(31),
    INTAKE(4),
    ALGAE_INTAKE_REEF(15),
    CLIMB(13),
    SAFE_MIDWAY(11.5);
    // CHANGE VALUES WHEN CAD FINISHES

    private double position = 0;

    private static final double EPSILON = ElevatorConstants.POSITION_TARGET_EPSILON;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }
  }

  private final LinearFilter supplyCurrentFilter;

  private LoggableMechanism3d loggableMechanism3dParent = null;

  private double filteredSupplyCurrentAmps = 0;

  private GenericSuperstructureIOInputsMotor2AutoLogged inputs2 = new GenericSuperstructureIOInputsMotor2AutoLogged();

  private boolean zeroing = false;

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.INTAKE);
    setControlMode(ControlMode.STOP);

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {
    superstructureIO.updateSecondaryInputs(inputs2);
    Logger.processInputs(name, inputs2);

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
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }

  // ------ LOGGABLE MECHANISM METHODS ------
  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(ElevatorConstants.ELEVATOR_BASE_3D_OFFSET)
        .plus(
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(getPosition())), // Add the current elevator's extension
                new Rotation3d(0, 0, 0))); // The elevator doesn't rotate, duh
  }

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
}
