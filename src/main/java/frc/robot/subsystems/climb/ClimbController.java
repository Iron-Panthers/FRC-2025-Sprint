package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.climbPivot.ClimbPivot;
import frc.robot.subsystems.climb.climbPivot.ClimbPivot.ClimbPivotTarget;
import frc.robot.subsystems.climb.climbRollers.ClimbRollers;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensors;
import org.littletonrobotics.junction.Logger;

public class ClimbController extends SubsystemBase {

  public enum ClimbState {
    /** Stowed state of the climb mech */
    IDLE,
    /** State for intaking the cage */
    INTAKE,
    /** Pose for flicking out the coral from the climb mech (if it is in the robot) */
    CLEAR,
    /** The actual action of climbing */
    CLIMB,
    /** No voltage */
    STOP_INTAKE,
    STOP_CLIMB;
  }

  private final ClimbRollers climbRollers;

  private final ClimbPivot climbPivot;

  private final ClimbSensors climbSensors;

  private ClimbState targetState = ClimbState.IDLE;

  public ClimbController(
      ClimbRollers climbRollers, ClimbPivot climbPivot, ClimbSensors climbSensors) {
    this.climbRollers = climbRollers;
    this.climbPivot = climbPivot;
    this.climbSensors = climbSensors;
  }

  @Override
  public void periodic() {
    climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);
        climbPivot.setPositionTarget(ClimbPivotTarget.BOTTOM);
      }
      case INTAKE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.INTAKE);
        climbPivot.setPositionTarget(ClimbPivotTarget.STOW);
      }
      case CLEAR -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.HOLD);
        climbPivot.setPositionTarget(ClimbPivotTarget.CLEAR);
      }
      case CLIMB -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.HOLD);
        climbPivot.setPositionTarget(ClimbPivotTarget.TOP);
      }
      case STOP_INTAKE -> {
        climbPivot.setControlMode(ControlMode.STOP);
        climbRollers.setVoltageTarget(ClimbRollers.Target.INTAKE);
      }
      case STOP_CLIMB -> {
        climbPivot.setControlMode(ControlMode.STOP);
        climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);
      }
    }

    // periodics
    climbPivot.periodic();
    climbRollers.periodic();
    climbSensors.periodic();
    if (climbPivot.getPosition() > ClimbPivotTarget.TOP.getPosition()) {
      setTargetState(ClimbState.STOP_CLIMB);
    }
    if (climbPivot.reachedTarget()
        && targetState != ClimbState.STOP_INTAKE
        && targetState != ClimbState.STOP_CLIMB) {
      setTargetState(
          targetState == ClimbState.INTAKE ? ClimbState.STOP_INTAKE : ClimbState.STOP_CLIMB);
    }

    Logger.recordOutput("Climb/TargetState", targetState);
  }

  /** Flick the climb to let coral fall out */
  public Command clearCoralCommand() {
    return new SequentialCommandGroup(
        // Wait until we get to the clear position
        new FunctionalCommand(
            () -> {
              climbPivot.setPositionTarget(ClimbPivotTarget.CLEAR);
            },
            () -> {},
            (e) -> {},
            climbPivot::reachedTarget),

        // Then just go back up to stow
        new InstantCommand(
            () -> {
              climbPivot.setPositionTarget(ClimbPivotTarget.STOW);
            }));
  }

  public boolean climbPivotHitCage() {
    return climbPivot.hitCage();
  }

  public ClimbPivotTarget getClimbPivotTarget() {
    return climbPivot.getPositionTarget();
  }

  public void setPivotClimbTarget(ClimbPivotTarget target) {
    climbPivot.setControlMode(ControlMode.POSITION);
    climbPivot.setPositionTarget(target);
  }

  public void setStopped(boolean stopped) {
    climbPivot.setControlMode(ControlMode.STOP);
  }

  public ClimbState getTargetState() {
    return targetState;
  }

  public void setTargetState(ClimbState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(ClimbState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }
}
