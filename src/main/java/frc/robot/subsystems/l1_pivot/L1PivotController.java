package frc.robot.subsystems.l1_pivot;

import static frc.robot.subsystems.l1_pivot.L1PivotConstants.ZEROING_VOLTAGE_THRESHOLD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.l1_pivot.L1Pivot.L1PivotTarget;
import org.littletonrobotics.junction.Logger;

public class L1PivotController extends SubsystemBase {

  public enum L1PivotState {
    /** Storing at the 90 degree up state */
    STOW,
    /** Going down to push the coral out */
    SCORE_L1,
    /** Going to the climb position */
    CLIMB,
    /** Zeroing the subsystem */
    ZEROING
  }

  private L1PivotState targetState;

  private final L1Pivot l1Pivot;

  public L1PivotController(L1Pivot l1Pivot) {
    this.l1Pivot = l1Pivot;
    this.targetState = L1PivotState.ZEROING;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (targetState) {
      case STOW -> {
        l1Pivot.setPositionTarget(L1PivotTarget.STOW);
      }
      case SCORE_L1 -> {
        l1Pivot.setPositionTarget(L1PivotTarget.L1_SCORE);
      }
      case CLIMB -> {
        l1Pivot.setPositionTarget(L1PivotTarget.CLIMB);
      }
      case ZEROING -> {
        l1Pivot.setZeroing(true);
        if (l1Pivot.getFilteredSupplyCurrentAmps() > ZEROING_VOLTAGE_THRESHOLD) {
          l1Pivot.setOffset();
          l1Pivot.setZeroing(false);
          setTargetState(L1PivotState.STOW);
        }
      }
    }

    l1Pivot.periodic();

    Logger.recordOutput("L1Pivot/TargetState", targetState);
  }

  public Command setTargetStateCommand(L1PivotState targetState) {
    return new InstantCommand(
            () -> {
              setTargetState(targetState);
            },
            this)
        .withTimeout(0.02)
        .andThen(new WaitUntilCommand(this::l1PivotReachedTarget));
  }

  /**
   * Gets the current state of the L1 pivot subsystem
   *
   * @return
   */
  public L1PivotState getTargetState() {
    return targetState;
  }

  public boolean l1PivotReachedTarget() {
    return l1Pivot.reachedTarget();
  }

  /**
   * Sets the L1 pivot state to the given target state
   *
   * @param targetState
   */
  public void setTargetState(L1PivotState targetState) {
    this.targetState = targetState;
  }

  public void setStopped(boolean stopped) {
    l1Pivot.setControlMode(ControlMode.STOP);
  }
}
