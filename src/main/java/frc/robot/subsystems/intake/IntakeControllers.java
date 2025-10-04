package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import org.littletonrobotics.junction.Logger;

public class IntakeControllers extends SubsystemBase {

  public enum RollerState {
    IDLE,
    INTAKE,
    FORCE_INTAKE,
    EJECT_TOP,
    EJECT_L1,
    EJECT_L2,
    EJECT_L3,
    HOLD,
    POSITIVE,
    NEGATIVE
  }

  private final IntakeRollers intake;
  // private double ejectTime = 0;
  private double intakeTime = 0;

  private RollerState targetState = RollerState.IDLE;

  public IntakeControllers(IntakeRollers intake) {
    this.intake = intake;
  }

  @Override
  public void periodic() {

    intake.setVoltageTarget(IntakeRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        intake.setVoltageTarget(IntakeRollers.Target.IDLE);
      }
      case INTAKE -> {
        intake.setVoltageTarget(IntakeRollers.Target.INTAKE);
      }
      case FORCE_INTAKE -> {
        intakeTime += 0.02;
        intake.setVoltageTarget(IntakeRollers.Target.INTAKE);
        if (intakeTime > 0.5) {
          this.targetState = RollerState.INTAKE;
          intakeTime = 0;
        }
      }
      case HOLD -> {
        intake.setVoltageTarget(IntakeRollers.Target.HOLD);
      }
      case EJECT_TOP -> {
        intake.setVoltageTarget(IntakeRollers.Target.EJECT_TOP);
      }
      case EJECT_L1 -> {
        intake.setVoltageTarget(IntakeRollers.Target.EJECT_L1);
      }
      case EJECT_L2 -> {
        intake.setVoltageTarget(IntakeRollers.Target.EJECT_L2);
      }
      case EJECT_L3 -> {
        intake.setVoltageTarget(IntakeRollers.Target.EJECT_L3);
      }
      case POSITIVE -> {
        intake.setVoltageTarget(IntakeRollers.Target.POSITIVE);
      }
      case NEGATIVE -> {
        intake.setVoltageTarget(IntakeRollers.Target.NEGATIVE);
      }
    }

    intake.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public RollerState getTargetState() {
    return targetState;
  }

  public void setTargetState(RollerState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(RollerState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }
}
