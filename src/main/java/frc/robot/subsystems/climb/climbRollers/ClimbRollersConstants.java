package frc.robot.subsystems.climb.climbRollers;

import frc.robot.Constants;

// TODO: Change ALLLLL of these constants as well
public class ClimbRollersConstants {
  public static final int ID =
      switch (Constants.getRobotType()) {
        case COMP -> 26;
        case SIM -> 26;
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
        case COMP -> false;
        case SIM -> false;
        default -> false;
      };
  public static final boolean BRAKE =
      switch (Constants.getRobotType()) {
        default -> false;
      };
  public static final double REDUCTION =
      switch (Constants.getRobotType()) {
        case COMP -> 1;
        case SIM -> 1;
        default -> 1;
      };

  public static final double MOI = 0.000105;
}
