package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class SuperstructureController extends SubsystemBase {

    /**
     * Enum for the different states of the superstructure
     * each state represents a different position or configuration of the
     * superstructure (Arm and Elevator)
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
         * Gets the target pose for the superstructure state
         * The arm direction part of the target pose tells future logic weather it
         * matters (CLOCKWISE, COUNTERCLOCKWISE) or doesn't (BOTH)
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

    /**
     * Enum for the direction the arm should move when going to a position
     */
    public enum ArmDirection {
        /** Clockwise when looking at the mechanism from the intake side */
        CLOCKWISE,
        /** Counter Clockwise when looking at the mechanism from the intake side */
        COUNTERCLOCKWISE,
        /**
         * The Arm can move in either direction and chooses the most optimal path
         * Basically hands off control to future logic
         */
        BOTH
    }

    /**
     * Record for the pose of the superstructure
     * 
     * @param elevatorHeight height of the elevator in meters
     * @param armAngle       angle of the arm in degrees
     * @param armDirection   direction the arm should move when going to a position
     */
    record SuperstructurePose(double elevatorHeight, double armAngle, ArmDirection armDirection) {
    };

    /**
     * Record for the physical constraints of the superstructure
     * 
     * @param minElevatorHeight minimum height of the elevator in meters
     * @param maxElevatorHeight maximum height of the elevator in meters
     * @param minArmAngle       minimum angle of the arm in degrees -- centered
     *                          around 0/360
     *                          being bottom of the elevator
     * @param maxArmAngle       maximum angle of the arm in degrees -- centered
     *                          around 0/360
     *                          being the bottom of the elevator
     */
    record SuperstructureConstraints(double minElevatorHeight, double maxElevatorHeight,
            double minArmAngle, double maxArmAngle) {
    };

    private SuperstructureState currentState = SuperstructureState.STOW;

    public SuperstructureState getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SuperstructureState state) {
        this.currentState = state;
    }

    private Elevator elevator;
    private Arm arm;

    /**
     * Constructor for the superstructure controller
     * 
     * @param elevator the elevator subsystem to control
     * @param arm      the arm subsystem to control
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
        SuperstructurePose targetPose = getTargetSuperstructurePose(); // gets the target pose for the superstructure
        elevator.setPositionTargetManual(targetPose.elevatorHeight);
        arm.setPositionTargetManual(targetPose.armAngle);
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
     * Get the target pose for the current target state
     * 
     * @return The modified superstructure target pose based on the current state
     *         and the physical constraints of the mechanism
     */
    public SuperstructurePose getTargetSuperstructurePose() {
        // 1. Calculate the Min and Max heights and angles for the elevator and pivot
        SuperstructureConstraints constraints = getSuperstructureConstraints();

        // 2. Clamp the elevator target height between the min and max
        double targetElevatorHeight = currentState.getTargetPose().elevatorHeight;

        // 3. Clamp the arm target angle between the min and max

        // 4. Figure out what direction the arm should be allowed to move

        return new SuperstructurePose(0, 0, null);
    }

    public SuperstructureConstraints getSuperstructureConstraints() {
        return new SuperstructureConstraints(0, 1, 0, 180);
    }

}
