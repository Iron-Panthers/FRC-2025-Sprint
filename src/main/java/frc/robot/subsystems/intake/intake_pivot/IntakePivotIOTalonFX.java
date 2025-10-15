package frc.robot.subsystems.intake.intake_pivot;

import static frc.robot.subsystems.intake.intake_pivot.IntakePivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;
import org.littletonrobotics.junction.AutoLogOutput;

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

  @AutoLogOutput(key = "Intake/IntakePivot/ModdedRotations")
  public double moddedRotations;

  @Override
  public void runPosition(double position) {

    position /= 360;
    moddedRotations =
        position
            - (talon.getPosition().getValueAsDouble()
                // + 0.1
                - ((talon.getPosition().getValueAsDouble()) % (1 / 2.25)));
    // - 0.1; // calculates how much the fricking encoder is off by (so sad🥲)
    super.runPosition(moddedRotations);
  }
}
