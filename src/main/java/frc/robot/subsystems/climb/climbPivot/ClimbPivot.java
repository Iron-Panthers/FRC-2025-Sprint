package frc.robot.subsystems.climb.climbPivot;

import static frc.robot.subsystems.climb.climbPivot.ClimbPivotConstants.INDUCTION_PORT_NUMBER;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.rgb.RGB.RGBMessages;
import org.littletonrobotics.junction.Logger;

// TODO: Change tuning
public class ClimbPivot extends GenericSuperstructure<ClimbPivot.ClimbPivotTarget> {
  public enum ClimbPivotTarget implements GenericSuperstructure.PositionTarget {

    // "Bottom" is ready to ram into cage
    BOTTOM(0.07),

    // "Top" is apex of climb
    TOP(0.307),

    // To get coral out
    CLEAR(-0.15),

    // When not climbing
    STOW(0.25);

    private double position = 0;
    private static final double EPSILON = ClimbPivotConstants.POSITION_TARGET_EPSILON;

    private ClimbPivotTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }
  }

  // induction sensor
  private DigitalInput inductionSensor;

  // run tino the cage - sensor triggers - flash leds to tell driver - button
  // presses : reels it in
  // or out
  // CAN'T BACKOUT
  // set position for intake in a cage, a button to climb up or down
  public ClimbPivot(ClimbPivotIO io) {
    super("Climb Pivot", io);
    inductionSensor = new DigitalInput(INDUCTION_PORT_NUMBER);
    setPositionTarget(ClimbPivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  // checks if the sensor has hit the cage
  public boolean hitCage() {
    return !inductionSensor.get();
  }

  @Override
  public void periodic() {

    super.periodic();
    Logger.recordOutput("Superstructure/Climb/Climb Pivot/Hit Cage?", hitCage());
    Logger.recordOutput("Superstructure/Climb/Climb Pivot/Climb Pivot State", getPositionTarget());
    SmartDashboard.putBoolean("Has Cage?", hitCage());
    RGBMessages.CLIMB.setIsExpired(!hitCage());
  }
}
