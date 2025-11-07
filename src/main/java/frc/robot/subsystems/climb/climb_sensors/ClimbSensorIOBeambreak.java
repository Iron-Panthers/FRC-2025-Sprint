package frc.robot.subsystems.climb.climb_sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorIO.ClimbSensorIOInputs;

public class ClimbSensorIOBeambreak implements ClimbSensorIO {
  private final DigitalInput beambreak;
  private final boolean sensorInverted;

  public ClimbSensorIOBeambreak() {
    beambreak = new DigitalInput(ClimbSensorsConstants.SENSOR_PORT);
    this.sensorInverted = ClimbSensorsConstants.SENSOR_INVERTED;
  }

  @Override
  public void updateInputs(ClimbSensorIOInputs inputs) {
    inputs.connected = true;
    inputs.triggered = beambreak.get() ^ sensorInverted;
  }
}
