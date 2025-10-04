package frc.robot.subsystems.intake.intakeRollers;

import static frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {

  private final FlywheelSim intakeRollersSim;

  public IntakeRollersIOSim() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    intakeRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    intakeRollersSim.setInputVoltage(appliedVoltage);
    intakeRollersSim.update(0.02);

    double rotations = 0; // can't really be simulated
    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() * REDUCTION;

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRads = rotations;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
  }
}
