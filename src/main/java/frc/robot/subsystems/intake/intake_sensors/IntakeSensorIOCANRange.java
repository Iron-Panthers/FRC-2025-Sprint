package frc.robot.subsystems.intake.intake_sensors;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class IntakeSensorIOCANRange implements IntakeSensorIO {

  private final CANrange canRange;
  // private final StatusSignal<Distance> distance;
  // private final StatusSignal<Boolean> isDetected;

  public IntakeSensorIOCANRange(int id) {

    canRange = new CANrange(id);
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.045;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;
    canRange.getConfigurator().apply(config);

    // // FIXME: do we really need this?
    // canRange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeSensorIOInputs inputs) {
    inputs.distance = canRange.getDistance().getValueAsDouble();
    inputs.connected = canRange.isConnected();
    inputs.isDetected = canRange.getIsDetected().getValue();
  }
}
