package frc.robot.subsystems.intake.intake_sensors;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class IntakeSensors {

  private IntakeSensorIO intakeSensorIO1;
  private IntakeSensorIO intakeSensorIO2;
  private LinearFilter filter1;
  private double filteredDistance1;
  private LinearFilter filter2;
  private double filteredDistance2;
  private IntakeSensorIOInputsAutoLogged inputs2 = new IntakeSensorIOInputsAutoLogged();
  private IntakeSensorIOInputsAutoLogged inputs1 = new IntakeSensorIOInputsAutoLogged();

  public IntakeSensors(IntakeSensorIO intakeSensorIO1, IntakeSensorIO intakeSensorIO2) {
    this.intakeSensorIO1 = intakeSensorIO1;
    this.intakeSensorIO2 = intakeSensorIO2;
    this.filter1 = LinearFilter.movingAverage(40);
    this.filter2 = LinearFilter.movingAverage(40);
  }

  public void periodic() {
    filteredDistance1 = this.filter1.calculate(inputs1.distance);
    filteredDistance2 = this.filter2.calculate(inputs2.distance);
    intakeSensorIO1.updateInputs(inputs1);
    intakeSensorIO2.updateInputs(inputs2);
    Logger.processInputs("Intake/IntakeSensors1", inputs1);
    Logger.processInputs("Intake/IntakeSensors2", inputs2);

    RobotState.getInstance().updateSensorsTriggered(sensorsTriggered());
  }

  // FIXME: what in the actual fuck -- bruce
  /**
   * Sensor 2 is never triggered by itself if sensor1 is triggered but not sensor2, it returns 6 if
   * sensor1 is not triggered but sensor2 is, it returns 7 if both are triggered, it retruns 13;
   */
  public int filteredSensorsTriggered() {
    int output = 0;
    if (filteredDistance1 < 0.055) {
      output += 6;
    }
    if (filteredDistance2 < 0.055) {
      output += 7;
    }
    return output;
  }

  public int sensorsTriggered() {
    int output = 0;
    if (inputs1.isDetected) {
      output += 6;
    }
    if (inputs2.isDetected) {
      output += 7;
    }
    return output;
  }
}
