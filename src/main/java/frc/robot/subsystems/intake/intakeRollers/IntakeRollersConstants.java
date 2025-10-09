package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.Constants;

public class IntakeRollersConstants {
  public static final int ID =
      switch (Constants.getRobotType()) {
        case COMP -> 45;
        case SIM -> 45;
        case PRACTICE -> 45;
        default -> 0;
      };
  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case COMP -> 40;
        case SIM -> 40;
        default -> 40;
      };
  public static final boolean INVERTED =
      switch (Constants.getRobotType()) {
        case COMP -> true;
        case SIM -> true;
        case PRACTICE -> true;
        default -> true;
      };
  public static final boolean BRAKE =
      switch (Constants.getRobotType()) {
        default -> true;
      };
  public static final double REDUCTION =
      switch (Constants.getRobotType()) {
        case COMP -> 5;
        case SIM -> 5;
        default -> 1;
      };

  public static final double MOI = 0.000105;
}
