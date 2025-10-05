package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;

import org.littletonrobotics.junction.Logger;

public class SuperstructureController extends SubsystemBase {

  /**
   * Enum for the different states of the superstructure each state represents a different position
   * or configuration of the superstructure (Arm and Elevator)
   */
  public enum SuperstructureState {
    STOW(null),
    L1(null),
    L2(null),
    L3(null),
    L4(null),
    ALGAE_INTAKE(null),
    BARGE(null); // TODO ADD MORE STATES AND DOCUMENT THEM

    private final SuperstructurePose targetPose;

    /**
     * Gets the target pose for the superstructure state The arm direction part of the target pose
     * tells future logic weather it matters (CLOCKWISE, COUNTERCLOCKWISE) or doesn't (BOTH)
     *
     * @return the target pose for this superstructure state
     */
    public SuperstructurePose getTargetPose() {
      return targetPose;
    }

    private SuperstructureState(SuperstructurePose targetPose) {
      this.targetPose = targetPose;
    }
  }

  /** Enum for the direction the arm should move when going to a position */
  public enum ArmDirection {
    /** Clockwise when looking at the mechanism from the intake side */
    CLOCKWISE,
    /** Counter Clockwise when looking at the mechanism from the intake side */
    COUNTERCLOCKWISE,
    /**
     * The Arm can move in either direction and chooses the most optimal path Basically hands off
     * control to future logic
     */
    BOTH
  }

  /**
   * Record for the pose of the superstructure
   */
  public class SuperstructurePose {
    public final Distance elevatorHeight;
    public final Angle armAngle;
    public final ArmDirection armDirection;

    /**
     *Constructor for the superstructure pose
     *
     * @param elevatorHeight height of the elevator in meters
     * @param armAngle angle of the arm in degrees
     * @param armDirection direction the arm should move when going to a position
     */
    public SuperstructurePose(Distance elevatorHeight, Angle armAngle, ArmDirection armDirection) {
      this.elevatorHeight = elevatorHeight;
      this.armAngle = armAngle;
      this.armDirection = armDirection;
    }

    public Mechanism2d getAsMechanism2d() {
      Mechanism2d mech = new Mechanism2d(50, 50);
      mech.getRoot("Superstructure", 25, 0)
          .append(new MechanismLigament2d("Elevator", 5, elevatorHeight.in(Units.Inches)))
          .append(
              new MechanismLigament2d(
                  "Arm",
                  ArmConstants.ARM_LENGTH,
                  armAngle.in(Units.Degrees)));
      return mech;
    }
  }

  /**
   * Record for the physical constraints of the superstructure
   *
   * @param minElevatorHeight minimum height of the elevator in meters
   * @param maxElevatorHeight maximum height of the elevator in meters
   * @param minArmAngle minimum angle of the arm in degrees -- centered around 0/360 being bottom of
   *     the elevator
   * @param maxArmAngle maximum angle of the arm in degrees -- centered around 0/360 being the
   *     bottom of the elevator
   */
  record SuperstructureConstraints(
      Distance minElevatorHeight, Distance maxElevatorHeight, Angle minArmAngle, Angle maxArmAngle) {}
  ;

  /**
   * The current target state of the superstructure
   */
  private SuperstructureState currentState = SuperstructureState.STOW;

  /**
   * Get the current target state of the superstructure
   * @return The current target state of the superstructure
   */
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  /**
   * Set the current target state of the superstructure
   * @param state
   */
  public void setCurrentState(SuperstructureState state) {
    this.currentState = state;
  }

  // subsystems to control
  private Elevator elevator;
  private Arm arm;

  /**
   * Constructor for the superstructure controller
   *
   * @param elevator the elevator subsystem to control
   * @param arm the arm subsystem to control
   */
  public SuperstructureController(Elevator elevator, Arm arm) {
    // setup the subsystems
    this.elevator = elevator;
    this.arm = arm;

    // set the initial target state
    setCurrentState(SuperstructureState.STOW);
  }

  @Override
  public void periodic() {
    // 1. run state logic
    SuperstructurePose targetPose =
        getTargetSuperstructurePose(); // gets the target pose for the superstructure
    elevator.setPositionTargetManual(targetPose.elevatorHeight.in(Units.Inches));
    arm.setPositionTargetManual(targetPose.armAngle.in(Units.Degrees));
    // TODO: make sure the arm moves in the calculated direction

    // 2. update subsystem periodics
    elevator.periodic();
    arm.periodic();

    // 3. log outputs
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/TargetPose/ElevatorHeight", targetPose.elevatorHeight);
    Logger.recordOutput("Superstructure/TargetPose/ArmAngle", targetPose.armAngle);
    Logger.recordOutput("Superstructure/TargetPose/ArmDirection", targetPose.armDirection);
  }


  /**
   * Get the current pose of the superstructure based on the readings from the subsystems
   * @return The current superstructure pose
   */
  public SuperstructurePose getCurrentSuperstructurePose(){
    Distance currentElevatorHeight = Units.Inches.of(elevator.getPosition());
    Angle currentArmAngle = Units.Degrees.of(arm.getPosition());
    ArmDirection currentArmDirection = ArmDirection.BOTH; // TODO: figure out a way to get this
    return new SuperstructurePose(currentElevatorHeight, currentArmAngle, currentArmDirection);
  }

  /**
   * Get the target pose for the current target state
   *
   * @return The modified superstructure target pose based on the current state and the physical
   *     constraints of the mechanism
   */
  public SuperstructurePose getTargetSuperstructurePose() {
    // 1. Calculate the Min and Max heights and angles for the elevator and pivot
    SuperstructureConstraints constraints = getSuperstructureConstraints();

    // 2. Clamp the elevator target height between the min and max
    Distance targetElevatorHeight = clamp(
        currentState.getTargetPose().elevatorHeight,
        constraints.minElevatorHeight,
        constraints.maxElevatorHeight
    );

    // 3. Clamp the arm target angle between the min and max
    Angle targetArmAngle = clamp(
        currentState.getTargetPose().armAngle,
        constraints.minArmAngle,
        constraints.maxArmAngle
    );

    // 4. Figure out what direction the arm should be allowed to move
    ArmDirection targetArmDirection = currentState.getTargetPose().armDirection;

    return new SuperstructurePose(targetElevatorHeight, targetArmAngle, targetArmDirection);
  }

  /**
   * Clamp a measure between a min and max
   * @param <U> the unit used in the clamping
   * @param <M> the mesure type used in the clamping
   * @param val the value to clamp
   * @param min the minimum value
   * @param max the maximum value
   * @return the clamped value
   */
  public static <U extends Unit, M extends Measure<U>> M clamp(M val, M min, M max) {
    if (val.lt(min)) {
        return min;
    } else if (val.gt(max)) {
        return max;
    } else {
        return val;
    }
  }

  /**
   * Get the physical constraints of the superstructure
   * Based on where the arm and elevator currently are 
   *
   * @return The physical constraints of the superstructure
   */
  public SuperstructureConstraints getSuperstructureConstraints() {
    Distance minElevatorHeight = getMinElevatorHeight();
    Distance maxElevatorHeight = Units.Inches.of(ElevatorConstants.UPPER_EXTENSION_LIMIT);
    Pair<Angle, Angle> armAngleConstraints = getArmAngleConstraints(minElevatorHeight);
    Angle minArmAngle = armAngleConstraints.getFirst();
    Angle maxArmAngle = armAngleConstraints.getSecond();

    return new SuperstructureConstraints(minElevatorHeight, maxElevatorHeight, minArmAngle, maxArmAngle);
  }


  /**
   * Get the minimum height the elevator can be at based on the current position of the arm
   * @return The minimum height the elevator can be at
   */
  private Distance getMinElevatorHeight() {
    SuperstructurePose currentPose = getCurrentSuperstructurePose();
    Distance armHeightRelativeToElevator =
                Units.Inches.of( // convert the arm length to inches
                    ArmConstants.ARM_LENGTH
                        * Math.sin(currentPose.armAngle.in(Units.Radians))); // vertical component

    if (armHeightRelativeToElevator.compareTo(Units.Inches.of(0)) > 0) {
      // arm is above the elevator
      return Units.Inches.of(ElevatorConstants.MIN_HEIGHT); // return the min height of the mech because we know the arm is safe
    }

    // arm is below the elevator
    Distance minSafeHeight = Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION).plus(
        armHeightRelativeToElevator.unaryMinus() // get the inverse of the arm height relative to the elevator
    ); // add that to the min safe height for the mech and we get the min safe height for the elevator

    return minSafeHeight;
  }

  /**
   * Get the minimum and maximum angles the arm can be at based on the current height of the elevator
   * @return A pair containing the minimum and maximum angles the arm can be at
   */
  private Pair<Angle,Angle> getArmAngleConstraints() {
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    // We are essentially making a right triangle with three sides: the height on the elevator the arm has left, the length of the arm, and a horizontal distance we don't care about
    Distance heightOnElevator = currentPose.elevatorHeight.minus(Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION));
    Distance armLength = Units.Inches.of(ArmConstants.ARM_LENGTH);

    // case: if our elevator is already high enough that we don't really care what happens
    if (heightOnElevator.gt(armLength)) {
      // we have more height to play with then we have arm length, so we just return the full range
      return new Pair<Angle,Angle>(Units.Degrees.of(0), Units.Degrees.of(360));
    }

    // KEEP IN MIND THIS ONLY WORKS IF THE 0/360 POINT IS AT THE BOTTOM OF THE ELEVATOR
    // we then take the arccos of the ratio of these two previous values to get the angle that the arm can form
    double angleRad = Math.acos(heightOnElevator.div(armLength).in(Units.Value));
    Angle minAngle = Units.Radians.of(angleRad);
    Angle maxAngle = Units.Radians.of(2 * Math.PI - angleRad);

    return new Pair<Angle,Angle>(minAngle, maxAngle);
  }
}
