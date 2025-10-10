package frc.robot.subsystems.intake.intake_sensors;

public class IntakeSensors {

  private IntakeSensorsIO intakeSensorsIO;
  // creates separate autologged class for the values
  // and logs all the inputs in advantage kit
  private IntakeSensorsIOInputsAutoLogged inputs = new IntakeSensorsIOInputsAutoLogged();

  public IntakeSensors(IntakeSensorsIO intakeSensorsIO) {}
}
