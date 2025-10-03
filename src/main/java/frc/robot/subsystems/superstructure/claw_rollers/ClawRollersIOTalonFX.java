package frc.robot.subsystems.superstructure.claw_rollers;

import static frc.robot.subsystems.superstructure.claw_rollers.ClawRollersConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ClawRollersIOTalonFX extends GenericRollersIOTalonFX implements ClawRollersIO {

  public ClawRollersIOTalonFX() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
  }
}
