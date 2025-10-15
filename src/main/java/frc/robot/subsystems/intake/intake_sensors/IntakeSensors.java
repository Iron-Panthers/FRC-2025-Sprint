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
    intakeSensorsIO2.updateInputs(inputs2);
    Logger.processInputs("Intake/IntakeSensors1", inputs1);
    Logger.processInputs("Intake/IntakeSensors2", inputs2);
    RobotState.getInstance().updateSensorsTriggered(sensorsTriggered());
  }

  // Sensor 2 is never triggered by itself
  // if sensor1 is triggered but not sensor2, it returns 6
  // if sensor1 is not triggered but sensor2 is, it returns 7
  // if both are triggered, it retruns 13;
  public int sensorsTriggered() {
    int output = 0;
    if (inputs1.distance < 0.5) {
      output += 6;
    }
    if (inputs2.distance < 0.5) {
      output += 7;
    }
    return output;
  }
}
