package frc.robot.subsystems.intake.intake_sensors;

// we don't really need this class rn so we can fix it later
public class IntakeSensorIOSim implements IntakeSensorIO {
  @Override
  public void updateInputs(IntakeSensorIOInputs inputs) {
    inputs.distance = 0;
    inputs.connected = false;
  }
}
