package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawRollers.ClawRollersTarget;
import org.littletonrobotics.junction.Logger;

public class ClawRollersController extends SubsystemBase {
  public enum ClawState {
    IDLE,
    INTAKE,
    HOLD,
    EJECT_TOP,
    EJECT_L3,
    EJECT_L1,
    EJECT_L2;
  }

  private boolean stop = false;
  private ClawState targetState = ClawState.IDLE;

  private final ClawRollers clawRollers;

  public ClawRollersController(ClawRollers clawRollers) {
    this.clawRollers = clawRollers;
    clawRollers.setVoltageTarget(ClawRollersTarget.HOLD);
  }

  public void setClawTarget(ClawState targetState) {
    this.targetState = targetState;
  }

  @Override
  public void periodic() {
    if (!stop) {
      switch (targetState) {
        case EJECT_L1 -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_L1);
        }
        case EJECT_L2 -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_L2);
        }
        case EJECT_L3 -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_L3);
        }
        case EJECT_TOP -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_TOP);
        }
        case HOLD -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.HOLD);
        }
        case INTAKE -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.INTAKE);
          if (clawRollers.getFilteredCurrent() > ClawRollersConstants.INTAKE_CURRENT_THRESHOLD) {
            setClawTarget(ClawState.HOLD);
          }
        }
        case IDLE -> {
          clawRollers.setVoltageTarget(ClawRollersTarget.IDLE);
        }
      }
    } else {
      clawRollers.setVoltageTarget(ClawRollersTarget.HOLD);
    }

    clawRollers.periodic();

    Logger.recordOutput("ClawRollers/targetState", targetState);
    Logger.recordOutput("ClawRollers/Filtered Supply Current", clawRollers.getFilteredCurrent());
  }
}
