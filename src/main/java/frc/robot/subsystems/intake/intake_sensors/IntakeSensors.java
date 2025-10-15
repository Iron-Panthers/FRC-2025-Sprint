package frc.robot.subsystems.intake.intake_sensors;

import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class IntakeSensors {

  private IntakeSensorsIO intakeSensorsIO1;
  private IntakeSensorsIO intakeSensorsIO2;
  private IntakeSensorsIOInputsAutoLogged inputs2 = new IntakeSensorsIOInputsAutoLogged();
  private IntakeSensorsIOInputsAutoLogged inputs1 = new IntakeSensorsIOInputsAutoLogged();

  public IntakeSensors(IntakeSensorsIO intakeSensorsIO1, IntakeSensorsIO intakeSensorsIO2) {
    this.intakeSensorsIO1 = intakeSensorsIO1;
    this.intakeSensorsIO2 = intakeSensorsIO2;
  }

  public void updateInputs() {
    intakeSensorsIO1.updateInputs(inputs1);
    intakeSensorsIO1.updateInputs(inputs2);
    Logger.processInputs("Intake/IntakeSensors1", inputs1);
    Logger.processInputs("Intake/IntakeSensors2", inputs2);
    RobotState.getInstance().updateNumSensorsTriggered(numSensorsTriggered());
  }

  // Sensor 2 is never triggered by itself
  public int numSensorsTriggered() {
    if (inputs1.distance < 0.5 && inputs2.distance < 0.5) {
      return 2;
    } else if (inputs1.distance < 0.5) {
      return 1;
    }
    return 0;
  }
}
