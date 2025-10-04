package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeRollers extends GenericRollers<IntakeRollers.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(4),
    HOLD(0),
    EJECT(-4);

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
