package frc.robot.subsystems.L1Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.L1Pivot.L1Pivot.L1PivotTarget;

public class L1PivotController extends SubsystemBase {
private final L1Pivot l1Pivot;
  public L1PivotController(L1Pivot l1Pivot) {
    this.l1Pivot = l1Pivot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    l1Pivot.periodic();
  }

  public Command setPositionTargetCommand(L1PivotTarget target) {
    return new InstantCommand(
        () -> {
          l1Pivot.setPositionTarget(target);
        });
  }


  public L1PivotTarget getL1PivotTarget() {
    return l1Pivot.getPositionTarget();
  }

  public void setL1PivotTarget(L1PivotTarget target) {
    l1Pivot.setControlMode(ControlMode.POSITION);
    l1Pivot.setPositionTarget(target);
  }

  public void setStopped(boolean stopped) {
    l1Pivot.setControlMode(ControlMode.STOP);
  }
}
