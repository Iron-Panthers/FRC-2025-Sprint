package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import org.littletonrobotics.junction.Logger;

public class IntakeControllers extends SubsystemBase {

  public enum RollerState {
    IDLE,
    INTAKE,
    EJECT,
    HOLD,
    L1,
    PASS;
  }

  private final IntakeRollers intakeRollers;
  private final IntakePivot intakePivot;
  // private double ejectTime = 0;

  private RollerState targetState = RollerState.IDLE;

  public IntakeControllers(IntakeRollers intakeRollers, IntakePivot intakePivot) {
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
  }

  @Override
  public void periodic() {

    intakeRollers.setVoltageTarget(IntakeRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.IDLE);
        intakePivot.setPositionTargetManual(intakePivot.getPosition()); // FIXME: I'm sorry Nora
      }
      case INTAKE -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.INTAKE);
        intakePivot.setPositionTarget(IntakePivot.IntakePivotTarget.INTAKE);
      }
      case EJECT -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.EJECT);
      }
      case HOLD -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.HOLD);
        intakePivot.setPositionTarget(IntakePivot.IntakePivotTarget.STOW);
      }
      case L1 -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.HOLD);
        intakePivot.setPositionTarget(IntakePivot.IntakePivotTarget.L1);
      }
      case PASS -> {
        intakePivot.setPositionTarget(IntakePivot.IntakePivotTarget.PASS);
        if (intakePivot.reachedTarget()) {
          intakeRollers.setVoltageTarget(IntakeRollers.Target.EJECT);
        }
      }
    }

    intakeRollers.periodic();

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
