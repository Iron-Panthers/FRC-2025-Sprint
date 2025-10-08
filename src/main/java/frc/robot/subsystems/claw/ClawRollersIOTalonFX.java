package frc.robot.subsystems.claw;

import static frc.robot.subsystems.claw.ClawRollersConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ClawRollersIOTalonFX extends GenericRollersIOTalonFX implements ClawRollersIO {

  public ClawRollersIOTalonFX() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
  }
}
