package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.Arm.ArmTarget;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class SuperstructureController extends SubsystemBase {

  /**
   * Enum for the different states of the superstructure each state represents a different position
   * or configuration of the superstructure (Arm and Elevator)
   */
  public enum SuperstructureState {
    STOW(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.L2, ArmTarget.PICKUP, ArmDirection.BOTH)),
    L1_RIGHT(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.L1, ArmTarget.STRAIGHT, ArmDirection.BOTH)),
    L1_LEFT(
        SuperstructurePose.fromTargetStates(ElevatorTarget.L1, ArmTarget.LEFT, ArmDirection.BOTH));
    // TODO: add more states and document them here

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

  /** Class for storing pose information from the superstructure */
  public static class SuperstructurePose {
    /** Height of the elevator in meters */
    public final Distance elevatorHeight;
    /**
     * Angle of the arm relative to the horizon (horizontal right when looking from intake side is 0
     * degrees)
     */
    public final Angle armAngle;
    /** Direction the arm should move when going to a position */
    public final ArmDirection armDirection;

    /**
     * Constructor for the superstructure pose
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

    public LoggedMechanism2d getAsMechanism2d() {
      LoggedMechanism2d mech = new LoggedMechanism2d(50, 50);
      mech.getRoot("Superstructure", 25, 0)
          .append(new LoggedMechanismLigament2d("Elevator", elevatorHeight.in(Units.Inches), 90))
          .append(
              new LoggedMechanismLigament2d(
                  "Arm", ArmConstants.ARM_LENGTH, armAngle.in(Units.Degrees) - 90));
      return mech;
    }

    /**
     * Constructs a SuperstructurePose from Elevator and Arm targets
     *
     * @param elevatorTarget
     * @param armTarget
     * @param armDirection
     * @return
     */
    public static SuperstructurePose fromTargetStates(
        ElevatorTarget elevatorTarget, ArmTarget armTarget, ArmDirection armDirection) {
      return new SuperstructurePose(
          Units.Inches.of(elevatorTarget.getPosition()),
          Units.Degrees.of(armTarget.getPosition()),
          armDirection);
    }
  }

  /**
   * Record for the physical constraints of the superstructure
   *
   * @param minElevatorHeight minimum height of the elevator in meters
   * @param maxElevatorHeight maximum height of the elevator in meters
   * @param minArmAngle minimum angle of the arm in degrees -- centered around 0/360 being directly
   *     right when looking from the intake side
   * @param maxArmAngle maximum angle of the arm in degrees -- centered around 0/360 being directly
   *     right when looking from the intake side
   */
  record SuperstructureConstraints(
      Distance minElevatorHeight,
      Distance maxElevatorHeight,
      Angle minArmAngle,
      Angle maxArmAngle) {}
  ;

  /** The current target state of the superstructure */
  private SuperstructureState superstructureState = SuperstructureState.STOW;

  /**
   * Get the current target state of the superstructure
   *
   * @return The current target state of the superstructure
   */
  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  /**
   * Set the current target state of the superstructure
   *
   * @param state
   */
  public void setSuperstructureState(SuperstructureState state) {
    this.superstructureState = state;
  }

  /**
   * Gets the relative angle to give to the motor controller to reach the given absolute angle
   *
   * @param absoluteAngle (0-360 degrees) centered at directly right when looking from the intake
   *     side
   * @param armDirection the direction the arm should move when going to the position
   * @return the relative angle to give to the motor controller
   */
  public double absoluteToRelativeTarget(
      Angle absoluteAngle, SuperstructurePose currentPose, ArmDirection armDirection) {

    // Get current raw encoder position (can be any value)
    double currentRawPosition = currentPose.armAngle.in(Units.Degrees);

    // Get the current absolute angle (0-360)
    double currentAbsoluteAngle = normalizeAngle(currentRawPosition);

    // Get the target absolute angle (0-360)
    double targetAbsoluteAngle = normalizeAngle(absoluteAngle.in(Units.Degrees));

    // Normalize delta to [-180, 180] range
    double deltaAngle = calculateShortestDeltaAngle(targetAbsoluteAngle, currentAbsoluteAngle);

    // Calculate the two possible targets
    double closestAngle = currentRawPosition + deltaAngle;

    double farthestAngle = currentRawPosition + deltaAngle + (deltaAngle > 0 ? -360.0 : 360.0);

    // Choose target based on direction preference
    return switch (armDirection) {
      case CLOCKWISE -> deltaAngle >= 0 ? farthestAngle : closestAngle;
      case COUNTERCLOCKWISE -> deltaAngle <= 0 ? farthestAngle : closestAngle;
      case BOTH -> closestAngle; // Use the shortest path
    };
  }

  /**
   * Normalizes an angle to be between 0-360 degrees
   *
   * @param angle The angle to normalize
   * @return The normalized angle between 0-360 degrees
   */
  private double normalizeAngle(double angle) {
    return ((angle % 360.0) + 360.0) % 360.0;
  }

  /**
   * Calculates the shortest delta between two angles
   *
   * @param target Target angle (0-360)
   * @param current Current angle (0-360)
   * @return Delta angle in range [-180, 180]
   */
  private double calculateShortestDeltaAngle(double target, double current) {
    // Normalize both angles to [0, 360) range first
    target = normalizeAngle(target);
    current = normalizeAngle(current);

    // Calculate the direct difference
    double deltaAngle = target - current;

    // Normalize to [-180, 180] range for shortest path
    if (deltaAngle > 180.0) {
      deltaAngle -= 360.0;
    } else if (deltaAngle < -180.0) {
      deltaAngle += 360.0;
    }

    return deltaAngle;
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
    setSuperstructureState(SuperstructureState.STOW);
  }

  @Override
  public void periodic() {
    // Run state logic and update targets
    updateTargets();

    // Update subsystem periodics
    updateSubsystemPeriodics();

    // Log data
    logData();
  }

  /** Updates targets for subsystems based on current state and constraints */
  private void updateTargets() {
    SuperstructurePose targetPose = getTargetSuperstructurePose();
    SuperstructurePose currentPose = getCurrentSuperstructurePose();
    elevator.setPositionTargetManual(targetPose.elevatorHeight.in(Units.Inches));
    arm.setPositionTargetManual(
        absoluteToRelativeTarget(targetPose.armAngle, currentPose, targetPose.armDirection));
  }

  /** Updates subsystem periodic methods */
  private void updateSubsystemPeriodics() {
    elevator.periodic();
    arm.periodic();
  }

  /** Logs relevant data about the superstructure */
  private void logData() {
    SuperstructurePose targetPose = getTargetSuperstructurePose();
    SuperstructurePose currentPose = getCurrentSuperstructurePose();
    SuperstructureConstraints constraints = getSuperstructureConstraints();

    // Log state
    Logger.recordOutput("Superstructure/SuperstructureState", superstructureState);

    // Log target pose data
    Logger.recordOutput("Superstructure/TargetPose/Mechanism2d", targetPose.getAsMechanism2d());
    Logger.recordOutput(
        "Superstructure/TargetPose/ElevatorHeight", targetPose.elevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/TargetPose/ArmAngle", targetPose.armAngle.in(Units.Degrees));
    Logger.recordOutput("Superstructure/TargetPose/ArmDirection", targetPose.armDirection);

    // Log current pose data
    Logger.recordOutput("Superstructure/CurrentPose/Mechanism2d", currentPose.getAsMechanism2d());
    Logger.recordOutput(
        "Superstructure/CurrentPose/ElevatorHeight", currentPose.elevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/CurrentPose/ArmAngle", currentPose.armAngle.in(Units.Degrees));
    Logger.recordOutput("Superstructure/CurrentPose/ArmDirection", currentPose.armDirection);

    // Log constraints
    Logger.recordOutput(
        "Superstructure/Constraints/MaxElevator", constraints.maxElevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/Constraints/MinElevator", constraints.minElevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/Constraints/MaxArm", constraints.maxArmAngle.in(Units.Degrees));
    Logger.recordOutput(
        "Superstructure/Constraints/MinArm", constraints.minArmAngle.in(Units.Degrees));
  }

  /**
   * Get the current pose of the superstructure based on the readings from the subsystems
   *
   * @return The current superstructure pose
   */
  public SuperstructurePose getCurrentSuperstructurePose() {
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
    // Get the prerequisite info for these calculations
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    // 1. Calculate the Min and Max heights and angles for the elevator and pivot
    SuperstructureConstraints constraints = getSuperstructureConstraints();
    Angle armMinimumAngle =
        Units.Degrees.of(normalizeAngle(constraints.minArmAngle.in(Units.Degrees)));
    Angle armMaximumAngle =
        Units.Degrees.of(normalizeAngle(constraints.maxArmAngle.in(Units.Degrees)));
    Angle currentNormalizedAngle =
        Units.Degrees.of(normalizeAngle(currentPose.armAngle.in(Units.Degrees)));

    // 2. Clamp the elevator target height between the min and max
    Distance targetElevatorHeight =
        clamp(
            superstructureState.getTargetPose().elevatorHeight,
            constraints.minElevatorHeight,
            constraints.maxElevatorHeight);

    Logger.recordOutput( // FIXME: Temporary logging for debugging in this function
        "Superstructure/DebugTargetPose/Target arm position",
        superstructureState.getTargetPose().armAngle.in(Units.Degrees));
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/Current arm position",
        currentPose.armAngle.in(Units.Degrees));
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/Normalized target arm position",
        normalizeAngle(superstructureState.getTargetPose().armAngle.in(Units.Degrees)));
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/Normalized current arm position",
        normalizeAngle(currentPose.armAngle.in(Units.Degrees)));

    // 3. Figure out what direction the arm should be allowed to move
    ArmDirection targetArmDirection = superstructureState.getTargetPose().armDirection;
    double shortestDeltaAngleToTarget =
        calculateShortestDeltaAngle(
            normalizeAngle(superstructureState.getTargetPose().armAngle.in(Units.Degrees)),
            normalizeAngle(currentPose.armAngle.in(Units.Degrees)));
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/Shortest delta angle to target",
        shortestDeltaAngleToTarget);
    if (targetArmDirection == ArmDirection.BOTH) {
      // see what direction is the most optimal direction and set our direction based
      // on that
      targetArmDirection =
          (shortestDeltaAngleToTarget >= 0)
              ? ArmDirection.COUNTERCLOCKWISE
              : ArmDirection.CLOCKWISE;
    }
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/Arm direction", targetArmDirection.toString());

    // 4. Modify the target arm pose based on the direction we want to go (if we
    // want to go clockwise we go to the nearest mod of the target angle in the
    // positive direction)
    Angle modifiedTargetAngle = superstructureState.getTargetPose().armAngle;
    if (targetArmDirection == ArmDirection.CLOCKWISE) {
      // if we are going clockwise, take the shortest delta angle and make it negative
      // and add it to the current angle
      double deltaAngle =
          (shortestDeltaAngleToTarget <= 0)
              ? shortestDeltaAngleToTarget
              : shortestDeltaAngleToTarget - 360.0;
      Logger.recordOutput("Superstructure/DebugTargetPose/Calculated delta angle", deltaAngle);
      modifiedTargetAngle =
          Units.Degrees.of(normalizeAngle(currentPose.armAngle.in(Units.Degrees)) + deltaAngle);
      Logger.recordOutput(
          "Superstructure/DebugTargetPose/Modified target angle",
          modifiedTargetAngle.in(Units.Degrees));
    } else if (targetArmDirection == ArmDirection.COUNTERCLOCKWISE) {
      // if we are going counterclockwise, take the shortest delta angle and
      // make it positive and add it to the current angle
      double deltaAngle =
          (shortestDeltaAngleToTarget >= 0)
              ? shortestDeltaAngleToTarget
              : shortestDeltaAngleToTarget + 360.0;
      Logger.recordOutput("Superstructure/DebugTargetPose/Calculated delta angle", deltaAngle);
      modifiedTargetAngle =
          Units.Degrees.of(normalizeAngle(currentPose.armAngle.in(Units.Degrees)) + deltaAngle);
      Logger.recordOutput(
          "Superstructure/DebugTargetPose/Modified target angle",
          modifiedTargetAngle.in(Units.Degrees));
    }

    // 5. fix the case where our current angle is between the
    // min and max angle but just in the wrong way (e.g. min = 350, max = 10,
    // current = 0)
    if (armMinimumAngle.gt(currentNormalizedAngle) && armMaximumAngle.lt(currentNormalizedAngle)) {
      modifiedTargetAngle =
          (Math.abs(armMinimumAngle.minus(currentNormalizedAngle).in(Units.Degrees))
                  < (Math.abs(currentNormalizedAngle.minus(armMaximumAngle).in(Units.Degrees))))
              ? armMinimumAngle
              : armMaximumAngle; // TODO: add some logging error here
    }

    // 6. Make sure that the min and max angles are actually less than and greater
    // than
    // the current angle
    if (armMinimumAngle.gt(currentNormalizedAngle)) {
      // make sure the minimum angle is less than the current angle
      armMinimumAngle = armMinimumAngle.minus(Units.Degrees.of(360.0));
    }
    if (armMaximumAngle.lt(currentNormalizedAngle)) {
      // make sure the maximum angle is greater than the current angle
      armMaximumAngle = armMaximumAngle.plus(Units.Degrees.of(360.0));
    }
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/ChangingMinMax/after min",
        armMinimumAngle.in(Units.Degrees));
    Logger.recordOutput(
        "Superstructure/DebugTargetPose/ChangingMinMax/after max",
        armMaximumAngle.in(Units.Degrees));

    // 7. Clamp the arm target angle between the min and max
    Angle targetArmAngle = clamp(modifiedTargetAngle, armMinimumAngle, armMaximumAngle);

    return new SuperstructurePose(targetElevatorHeight, targetArmAngle, targetArmDirection);
  }

  /**
   * Clamp a measure between a min and max
   *
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
   * Get the physical constraints of the superstructure Based on where the arm and elevator
   * currently are
   *
   * @return The physical constraints of the superstructure
   */
  public SuperstructureConstraints getSuperstructureConstraints() {
    Distance minElevatorHeight = getMinElevatorHeight();
    Distance maxElevatorHeight = Units.Inches.of(ElevatorConstants.UPPER_EXTENSION_LIMIT);
    Pair<Angle, Angle> armAngleConstraints = getArmAngleConstraints();
    Angle minArmAngle = armAngleConstraints.getFirst();
    Angle maxArmAngle = armAngleConstraints.getSecond();

    return new SuperstructureConstraints(
        minElevatorHeight, maxElevatorHeight, minArmAngle, maxArmAngle);
  }

  /**
   * Get the minimum height the elevator can be at based on the current position of the arm
   *
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
      return Units.Inches.of(
          ElevatorConstants
              .MIN_HEIGHT); // return the min height of the mech because we know the arm is safe
    }

    // arm is below the elevator
    Distance minSafeHeight =
        Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION)
            .plus(
                armHeightRelativeToElevator
                    .unaryMinus() // get the inverse of the arm height relative to the elevator
                ); // add that to the min safe height for the mech and we get the min safe height
    // for the elevator

    return minSafeHeight;
  }

  /**
   * Get the minimum and maximum angles the arm can be at based on the current height of the
   * elevator
   *
   * @return A pair containing the minimum and maximum angles the arm can be at
   */
  private Pair<Angle, Angle> getArmAngleConstraints() {
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    // Calculate available height above minimum safe height
    Distance heightOnElevator = calculateAvailableElevatorHeight(currentPose);
    Distance armLength = Units.Inches.of(ArmConstants.ARM_LENGTH);

    // If elevator is high enough, allow full range of motion
    if (heightOnElevator.gt(armLength)) {
      return new Pair<>(Units.Degrees.of(-90), Units.Degrees.of(270));
    }

    // Calculate angle constraints based on available height
    return calculateAngleConstraints(heightOnElevator, armLength);
  }

  /** Calculates the available height on the elevator for arm movement */
  private Distance calculateAvailableElevatorHeight(SuperstructurePose pose) {
    return pose.elevatorHeight.minus(
        Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION));
  }

  /** Calculates angle constraints based on available height and arm length */
  private Pair<Angle, Angle> calculateAngleConstraints(
      Distance heightOnElevator, Distance armLength) {
    // Calculate the minimum angle using arccos
    double angleRad = Math.acos(heightOnElevator.div(armLength).in(Units.Value));

    Angle minAngle = Units.Radians.of(angleRad);
    Angle maxAngle = Units.Radians.of(2 * Math.PI - angleRad);

    // Convert from bottom-relative to right-relative coordinates
    minAngle = minAngle.minus(Units.Degrees.of(90));
    maxAngle = maxAngle.minus(Units.Degrees.of(90));

    return new Pair<>(minAngle, maxAngle);
  }
}
