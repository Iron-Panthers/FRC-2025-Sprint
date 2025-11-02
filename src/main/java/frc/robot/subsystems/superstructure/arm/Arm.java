package frc.robot.subsystems.superstructure.arm;

import frc.robot.lib.generic_subsystems.superstructure.*;
import org.littletonrobotics.junction.Logger;

public class Arm extends GenericSuperstructure<Arm.ArmTarget> {
  public enum ArmTarget implements GenericSuperstructure.PositionTarget {
    TOP(90),
    PICKUP(-90),
    BOTTOM(-90),
    STRAIGHT(0),
    GROUND_ALGAE(-20),
    BARGE_RIGHT(45),
    LEFT(180);

    private double position;
    private static final double EPSILON = ArmConstants.POSITION_TARGET_EPSILON;

    private ArmTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }
  }

  public Arm(ArmIO io) {
    super("Arm", io);
    setPositionTarget(ArmTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Superstructure/Arm/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    double targetPosition =
        switch (controlMode) {
          case POSITION -> positionTarget.getPosition() / 360d;
          case POSITION_MANUAL -> super.positionTargetManual.orElse(0d) / 360d;
          case STOP -> inputs.positionRotations;
        };
    return Math.abs(inputs.positionRotations - targetPosition) <= positionTarget.getEpsilon();
  }

  /** Returns the position of the arm in DEGREES */
  public double getPosition() {
    return super.getPosition() * 360.0;
  }
}
