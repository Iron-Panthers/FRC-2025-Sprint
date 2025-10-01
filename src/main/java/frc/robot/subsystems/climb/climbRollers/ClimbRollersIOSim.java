package frc.robot.subsystems.climb.climbRollers;

import static frc.robot.subsystems.climb.climbRollers.ClimbRollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ClimbRollersIOSim extends GenericRollersIOSim implements ClimbRollersIO {

  private final FlywheelSim climbRollersSim;

  public ClimbRollersIOSim() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    climbRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), MOI, REDUCTION),
            DCMotor.getKrakenX60Foc(2));
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    climbRollersSim.setInputVoltage(appliedVoltage);
    climbRollersSim.update(0.02);

    double rotations = 0; // can't really be simulated
    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS = climbRollersSim.getAngularVelocityRadPerSec() * REDUCTION;

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRads = rotations;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
  }
}
