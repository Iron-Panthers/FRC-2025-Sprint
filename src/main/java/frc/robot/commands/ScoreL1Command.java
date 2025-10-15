// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.l1_pivot.L1PivotController;
import frc.robot.subsystems.l1_pivot.L1PivotController.L1PivotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL1Command extends SequentialCommandGroup {
  /** Creates a new ScoreL1Command. */
  public ScoreL1Command(IntakeController intakeController, L1PivotController l1PivotController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (intakeController.getTargetState() == IntakeController.IntakeState.HOLD){
      addCommands(
          l1PivotController.setTargetStateCommand(L1PivotState.STOW),
          intakeController.setTargetCommand(IntakeState.L1),
          new WaitCommand(.5),
          l1PivotController.setTargetStateCommand(L1PivotState.SCORE_L1),
          l1PivotController.setTargetStateCommand(L1PivotState.STOW));
    }
  }
}
