package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeRollers extends GenericRollers<IntakeRollers.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(4),
    HOLD(0),
    EJECT_TOP(-8),
    EJECT_L3(2),
    EJECT_L1(2),
    EJECT_L2(2.4);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public IntakeRollers(IntakeRollersIO intakeRollersIO) {
    super("IntakeRollers", intakeRollersIO);
  }
}
