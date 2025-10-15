package frc.robot.subsystems.intake.intake_sensors;

import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class IntakeSensors {

  private IntakeSensorIO intakeSensorIO1;
  private IntakeSensorIO intakeSensorIO2;
  private IntakeSensorIOInputsAutoLogged inputs2 = new IntakeSensorIOInputsAutoLogged();
  private IntakeSensorIOInputsAutoLogged inputs1 = new IntakeSensorIOInputsAutoLogged();

  public IntakeSensors(IntakeSensorIO intakeSensorIO1, IntakeSensorIO intakeSensorIO2) {
    this.intakeSensorIO1 = intakeSensorIO1;
    this.intakeSensorIO2 = intakeSensorIO2;
  }

  public void periodic() {
    intakeSensorIO1.updateInputs(inputs1);
    intakeSensorIO2.updateInputs(inputs2);
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
