package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.ClawRollersController;
import frc.robot.subsystems.intake.IntakeController;

public class StopRollersCommand extends ParallelCommandGroup {
  public StopRollersCommand(
      IntakeController intakeController, ClawRollersController clawRollersController) {
    addCommands(
        intakeController.setTargetStateCommand(IntakeController.IntakeState.IDLE),
        new InstantCommand(
            () -> clawRollersController.setClawTarget(ClawRollersController.ClawState.IDLE)));
  }
}
