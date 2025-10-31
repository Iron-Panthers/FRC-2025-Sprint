package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.utility.ElasticPID;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {

  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0.6),
    L1(11),
    L2(30),
    L3(5),
    L4(32.5),
    ALGAE_SCORE_PROCESSOR(5),
    ALGAE_SCORE_NET(30),
    TOP(40),
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

  private double filteredSupplyCurrentAmps = 0;

  private GenericSuperstructureIOInputsMotor2AutoLogged inputs2 =
      new GenericSuperstructureIOInputsMotor2AutoLogged();

  private boolean zeroing = false;

  ElasticPID elasticPID;

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.INTAKE);
    setControlMode(ControlMode.STOP);

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);

    elasticPID =
        new ElasticPID(
            io::setSlot0,
            GravityTypeValue.Elevator_Static,
            "Elevator",
            ElevatorConstants.GAINS,
            ElevatorConstants.MOTION_MAGIC_CONFIG);
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
    Logger.recordOutput("Superstructure/" + name + "/Zeroing", zeroing);

    // elasticPID.periodic();
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
}
