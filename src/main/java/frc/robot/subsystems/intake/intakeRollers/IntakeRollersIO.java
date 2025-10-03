package frc.robot.subsystems.intake.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeRollersIOInputs {
    public boolean connected = true;
    public double positionRads = 0;
    public double velocityRadsPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
  }

  default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}
}
