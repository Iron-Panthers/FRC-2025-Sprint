package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.climbRollers.ClimbRollers;
import org.littletonrobotics.junction.Logger;

public class ClimbController extends SubsystemBase {

  public enum ClimbState {
    IDLE,
    INTAKE,
    HOLD
  }

  private final ClimbRollers climbRollers;

  private ClimbState targetState = ClimbState.IDLE;

  public ClimbController(ClimbRollers climbRollers) {
    this.climbRollers = climbRollers;
  }

  @Override
  public void periodic() {
    climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.IDLE);
      }
      case INTAKE -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.INTAKE);
      }
      case HOLD -> {
        climbRollers.setVoltageTarget(ClimbRollers.Target.HOLD);
      }
    }

    climbRollers.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public ClimbState getTargetState() {
    return targetState;
  }

  public void setTargetState(ClimbState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(ClimbState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }
}
