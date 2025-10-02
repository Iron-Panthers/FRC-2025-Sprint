package frc.robot.subsystems.superstructure.arm;

import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;

public class ArmIOTalonFX extends GenericSuperstructureIOTalonFX implements ArmIO {

  public ArmIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(ARM_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(ARM_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withCANCoderID(ARM_CONFIG.canCoderID())
            .withCANCoderOffset(ARM_CONFIG.canCoderOffset())
            .withCANCoderDirection(CANCODER_DIRECTION));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }

  /**
   * Move move the arm to a position with the given degrees
   */
  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
