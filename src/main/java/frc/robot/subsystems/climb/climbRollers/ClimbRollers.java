package frc.robot.subsystems.climb.climbRollers;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ClimbRollers extends GenericRollers<ClimbRollers.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(4),
    HOLD(0);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public ClimbRollers(ClimbRollersIO climbRollersIO) {
    super("Climb Rollers", climbRollersIO);
  }
}
