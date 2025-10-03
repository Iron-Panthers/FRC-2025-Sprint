package frc.robot.subsystems.superstructure.claw_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class ClawRollers extends GenericRollers<ClawRollers.ClawRollersTarget> {
  public enum ClawRollersTarget implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(4),
    HOLD(0),
    EJECT_TOP(-8),
    EJECT_L3(2),
    EJECT_L1(2),
    EJECT_L2(2.4);

    private double volts;

    private ClawRollersTarget(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public ClawRollers(ClawRollersIO io) {
    super("ClawRollers", io);
  }
}
