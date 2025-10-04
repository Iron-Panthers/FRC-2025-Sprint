package frc.robot.subsystems.intake.intakePivot;

import static frc.robot.subsystems.intake.intakePivot.IntakePivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;

public class IntakePivotIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakePivotIO {

  public IntakePivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(INTAKE_PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(INTAKE_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withCANCoderID(INTAKE_PIVOT_CONFIG.canCoderID())
            .withCANCoderOffset(INTAKE_PIVOT_CONFIG.canCoderOffset())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withUpperExtensionLimit(UPPER_EXTENSION_LIMIT));

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

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
