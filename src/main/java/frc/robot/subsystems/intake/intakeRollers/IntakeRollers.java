package frc.robot.subsystems.intake.intakeRollers;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

public abstract class IntakeRollers<G extends IntakeRollers.VoltageTarget> {
  public interface VoltageTarget {
    double getVolts();
  }

  private LinearFilter filter;
  private double filteredCurrent;

  private final String name;
  private final IntakeRollersIO rollerIO;
  private IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  private G voltageTarget;

  public IntakeRollers(String name, IntakeRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    rollerIO.runVolts(voltageTarget.getVolts());
    Logger.recordOutput("Rollers/" + name + "/Target", voltageTarget.toString());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput("Rollers/" + name + "/FilteredCurrent", filteredCurrent);
  }

  public G getVoltageTarget() {
    return voltageTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public void setVoltageTarget(G voltageTarget) {
    this.voltageTarget = voltageTarget;
  }
}
