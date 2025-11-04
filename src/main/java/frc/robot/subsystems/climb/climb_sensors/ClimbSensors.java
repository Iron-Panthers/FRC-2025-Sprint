package frc.robot.subsystems.climb.climb_sensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorIO.ClimbSensorIOInputs;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIO;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIOInputsAutoLogged;

public class ClimbSensors {
    
  private ClimbSensorIO climbSensorIO;
  private ClimbSensorIOInputsAutoLogged inputs = new ClimbSensorIOInputsAutoLogged();

  public ClimbSensors(ClimbSensorIO climbSensorIO) {
    this.climbSensorIO = climbSensorIO;
  }

  public void periodic() {
    climbSensorIO.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeSensors", inputs);
  }

  /**
   * Returns true if the sensor is triggered
   * @param none -- there are no parameters in this function
   * @see ing would be nice (i'm blind btw)
   * @return true if the sensor is triggered
   */
  public boolean sensorsTriggered() {
    return inputs.triggered;
  }
}
