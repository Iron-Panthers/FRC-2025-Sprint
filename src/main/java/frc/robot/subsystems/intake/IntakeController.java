package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot.IntakePivotTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import org.littletonrobotics.junction.Logger;

public class IntakeController extends SubsystemBase {

  public enum IntakeState {
    /** Idle state -- directly up with no spin */
    IDLE,
    /** Intake state -- down and spinning to intake */
    INTAKE,
    /** Eject state -- up and spinning to intake in reverse to score in L1 */
    EJECT,
    /** Hold state -- up and holding the coral */
    HOLD,
    /** L1 state -- going to the L1 scoring state but not ejecting yet */
    L1,
    /** Pass state -- going to the pass position for the grabber to get the coral */
    PASS;
  }

  private IntakeState targetState = IntakeState.IDLE;

  private final IntakeRollers intakeRollers;
  private final IntakePivot intakePivot;

  public IntakeController(IntakeRollers intakeRollers, IntakePivot intakePivot) {
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
  }

  @Override
  public void periodic() {

    intakeRollers.setVoltageTarget(IntakeRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.IDLE);
        intakePivot.setPositionTarget(IntakePivotTarget.STOW);
      }
      case INTAKE -> {
        if (RobotState.getInstance().getSensorsTriggered() != 0) {
          targetState = IntakeState.HOLD;
        }
        intakeRollers.setVoltageTarget(IntakeRollers.Target.INTAKE);
        intakePivot.setPositionTarget(IntakePivotTarget.INTAKE);
      }
      case EJECT -> { // TODO: Figure out more robust eject logic
        intakeRollers.setVoltageTarget(IntakeRollers.Target.EJECT);
        intakePivot.setPositionTarget(IntakePivotTarget.L1);
      }
      case HOLD -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.HOLD);
        intakePivot.setPositionTarget(IntakePivotTarget.STOW);
      }
      case L1 -> {
        intakeRollers.setVoltageTarget(IntakeRollers.Target.HOLD);
        intakePivot.setPositionTarget(IntakePivotTarget.L1);
        if (intakeReachedTarget()) {
          intakeRollers.setVoltageTarget(IntakeRollers.Target.EJECT);
        }
      }
      case PASS -> {
        intakePivot.setPositionTarget(IntakePivotTarget.PASS);
        if (intakePivot.reachedTarget()) {
          intakeRollers.setVoltageTarget(IntakeRollers.Target.EJECT);
        }
      }
    }

    intakeRollers.periodic();
    intakePivot.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  /**
   * Checks if the intake pivot mechanism has reached its target position.
   *
   * @return {@code true} if the intake pivot has reached its target position, {@code false}
   *     otherwise.
   */
  public boolean intakeReachedTarget() {
    return intakePivot.reachedTarget();
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  public void setTargetState(IntakeState targetState) {
    this.targetState = targetState;
  }

  /**
   * Creates a command to set the target state of the intake system.
   *
   * @param target The desired {@link IntakeState} to set as the target state.
   * @return A {@link Command} that sets the target state and monitors when the intake reaches the
   *     target.
   *     <p>The command performs the following actions: - Initializes by setting the target state of
   *     the intake system. - Executes with no additional behavior during the command's active
   *     phase. - Cleans up with no specific actions upon command termination. - Ends when the
   *     intake system reaches the specified target state.
   */
  public Command setTargetCommand(IntakeState target) {
    return new InstantCommand(
            () -> {
              this.targetState = target;
            },
            this)
        .withTimeout(.02)
        .andThen(new WaitUntilCommand(this::intakeReachedTarget));
  }
}
