package frc.robot.subsystems.climb.climb_sensors;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorIO.ClimbSensorIOInputs;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIO.IntakeSensorIOInputs;

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
