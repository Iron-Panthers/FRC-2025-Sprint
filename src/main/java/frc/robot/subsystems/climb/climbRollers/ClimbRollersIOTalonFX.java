package frc.robot.subsystems.climb.climbRollers;

import static frc.robot.subsystems.climb.climbRollers.ClimbRollersConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ClimbRollersIOTalonFX extends GenericRollersIOTalonFX implements ClimbRollersIO {

  public ClimbRollersIOTalonFX() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
  }
}
