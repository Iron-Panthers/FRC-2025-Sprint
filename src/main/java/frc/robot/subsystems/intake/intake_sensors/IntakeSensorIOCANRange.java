package frc.robot.subsystems.intake.intake_sensors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorsIO.IntakeSensorsIOInputs;

public class IntakeSensorIOCANRange implements IntakeSensorsIO {

  private final CANrange canRange;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;

  public IntakeSensorIOCANRange(int id) {

    canRange = new CANrange(id);

    canRange.getConfigurator().apply(new CANrangeConfiguration());

    distance = canRange.getDistance();
    isDetected = canRange.getIsDetected();

    // FIXME: do we really need this?
    canRange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeSensorsIOInputs inputs) {}
}
