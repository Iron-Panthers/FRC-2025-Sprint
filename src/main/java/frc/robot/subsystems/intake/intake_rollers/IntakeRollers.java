package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeRollers extends GenericRollers<IntakeRollers.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0), // for not moving
    INTAKE(12), // for intaking the coral
    HOLD(2), // for holding the coral
    EJECT(-3), // for ejecting into L1
    PASS(1); // for passing to the grabber

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public IntakeRollers(IntakeRollersIO intakeRollersIO) {
    super("Intake Rollers", intakeRollersIO);
  }
}
