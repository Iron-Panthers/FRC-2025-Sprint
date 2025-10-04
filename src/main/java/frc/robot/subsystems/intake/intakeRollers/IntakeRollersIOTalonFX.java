package frc.robot.subsystems.intake.intakeRollers;

import static frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {

  public IntakeRollersIOTalonFX() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
  }
}
