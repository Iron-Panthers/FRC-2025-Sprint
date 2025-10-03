package frc.robot.subsystems.climb.climbPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.climbPivot.ClimbPivot.ClimbPivotTarget;

public class ClimbPivotController extends SubsystemBase {

  private final ClimbPivot climbPivot;
  /** Creates a new ClimbController. */
  public ClimbPivotController(ClimbPivot climbPivot) {
    this.climbPivot = climbPivot;
    // climb.setOffset();
    // climb.setPositionTarget(ClimbTarget.STOW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbPivot.periodic();
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
}
