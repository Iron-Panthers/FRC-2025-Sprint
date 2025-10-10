package frc.robot.subsystems.intake.intakeSensors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intake.intakeSensors.IntakeSensorsIO.IntakeSensorsIOInputs;

public class IntakeSensorIOCANRange implements IntakeSensorsIO {

  // sensor object and objects for the values we want to find
  private final CANrange canRange;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;

  public IntakeSensorIOCANRange() {
    /* fix the args or the id in here */
    canRange = new CANrange(0);
    // configure the CANrange
    canRange.getConfigurator().apply(new CANrangeConfiguration());

    // find the distance and if an object is being detected
    distance = canRange.getDistance();
    isDetected = canRange.getIsDetected();
    
    //FIXME: do we really need this?
    canRange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeSensorsIOInputs inputs) {
    
  }
}
