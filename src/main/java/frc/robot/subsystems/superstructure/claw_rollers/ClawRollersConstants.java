package frc.robot.subsystems.superstructure.claw_rollers;

import frc.robot.Constants;

public class ClawRollersConstants {
  public static final int ID =
      switch (Constants.getRobotType()) {
        case COMP -> 26; // TODO CHANGE
        case SIM -> 26; // TODO CHANGE
        default -> 0;
      };
  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case COMP -> 40; // TODO CHANGE
        case SIM -> 40; // TODO CHANGE
        default -> 40;
      };
  // TODO: change to InvertedValue
  public static final boolean INVERTED =
      switch (Constants.getRobotType()) {
        case COMP -> false; // TODO CHANGE
        case SIM -> false; // TODO CHANGE
        default -> false;
      };
  public static final boolean BRAKE =
      switch (Constants.getRobotType()) {
        default -> false;
      };
  public static final double REDUCTION =
      switch (Constants.getRobotType()) {
        case COMP -> 2.25; // TODO CHANGE
        case SIM -> 2.25; // TODO CHANGE
        default -> 1;
      };
  // TODO: Change default to bot being used in Comp
  public static final double ROLLER_CIRCUMFERENCE =
      switch (Constants.getRobotType()) {
        case COMP, SIM -> 0.16022122533; // TODO CHANGE
        default -> 1;
      };

  public static final double MOI = 0.06; // MAYBE CHANGE?
}
