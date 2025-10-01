package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class ArmIOSim extends GenericSuperstructureIOSim implements ArmIO {

  private final SingleJointedArmSim pivotSim;
  private final double reduction;

  public ArmIOSim() {
    super(ArmConstants.PIVOT_CONFIG.motorID());

    this.reduction = ArmConstants.PIVOT_CONFIG.reduction();

    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            ArmConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ArmConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ArmConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ArmConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ArmConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        ArmConstants.GAINS.kP(),
        ArmConstants.GAINS.kI(),
        ArmConstants.GAINS.kD(),
        ArmConstants.GAINS.kS(),
        ArmConstants.GAINS.kV(),
        ArmConstants.GAINS.kA(),
        ArmConstants.GAINS.kG(),
        ArmConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ArmConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ArmConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(16);

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    pivotSim.setInputVoltage(appliedVoltage);
    pivotSim.update(0.02);

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double rotations = pivotSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = pivotSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
    inputs.tempCelsius = 25.0; // Not simulated
  }

  @Override
  public void setOffset() {
    pivotSim.setState(0, 0);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
