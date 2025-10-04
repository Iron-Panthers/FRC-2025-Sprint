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
import org.littletonrobotics.junction.Logger;

public class ClimbController extends SubsystemBase {

  public enum ClimbState {
    IDLE,
    INTAKE,
    CLEAR,
    CLIMB; // The actual action of climbing
  }

  private final ClimbRollers climbRollers;

  private final ClimbPivot climbPivot;

  private ClimbState targetState = ClimbState.IDLE;

  public ClimbController(ClimbRollers climbRollers, ClimbPivot climbPivot) {
    this.climbRollers = climbRollers;
    this.climbPivot = climbPivot;
  }

  @Override
  public void periodic() {
    climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);
        climbPivot.setPositionTarget(ClimbPivot.ClimbPivotTarget.STOW);
      }
      case INTAKE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.INTAKE);
        climbPivot.setPositionTarget(ClimbPivot.ClimbPivotTarget.STOW);
      }
      case CLEAR -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.HOLD);
        climbPivot.setPositionTarget(ClimbPivot.ClimbPivotTarget.CLEAR);
      }
      case CLIMB -> {
        //need an if statement that checks if we are actually ready to climb
        climbRollers.setVoltageTarget(ClimbRollers.Target.HOLD);
        climbPivot.setPositionTarget(ClimbPivot.ClimbPivotTarget.TOP);
      }
    }
    
    climbPivot.periodic();

    climbRollers.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public Command setPositionTargetCommand(ClimbPivotTarget target) {
    return new InstantCommand(
        () -> {
          climbPivot.setPositionTarget(target);
        });
  }

   // Flick the climb to let coral fall out
  public Command clearCoral() {
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
