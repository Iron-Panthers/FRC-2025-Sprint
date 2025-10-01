package frc.robot.subsystems.L1Pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class L1PivotIOSim extends GenericSuperstructureIOSim implements L1PivotIO {

  private final SingleJointedArmSim l1PivotSim;
  private final double reduction;

  public L1PivotIOSim() {
    super(L1PivotConstants.L1_PIVOT_CONFIG.motorID());

    this.reduction = L1PivotConstants.L1_PIVOT_CONFIG.reduction();

    l1PivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            L1PivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            L1PivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            L1PivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            L1PivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            L1PivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        L1PivotConstants.GAINS.kP(),
        L1PivotConstants.GAINS.kI(),
        L1PivotConstants.GAINS.kD(),
        L1PivotConstants.GAINS.kS(),
        L1PivotConstants.GAINS.kV(),
        L1PivotConstants.GAINS.kA(),
        L1PivotConstants.GAINS.kG(),
        L1PivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        L1PivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        L1PivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    l1PivotSim.setInputVoltage(appliedVoltage);
    l1PivotSim.update(0.02);

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double rotations = l1PivotSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = l1PivotSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

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
    l1PivotSim.setState(0, 0);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
