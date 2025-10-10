package frc.robot.subsystems.intake.intake_sensors;

import org.littletonrobotics.junction.Logger;

public class IntakeSensors {

  private IntakeSensorsIO intakeSensorsIO;
  private final int index;
  private IntakeSensorsIOInputsAutoLogged inputs = new IntakeSensorsIOInputsAutoLogged();

  public IntakeSensors(IntakeSensorsIO intakeSensorsIO, int index) {
    this.intakeSensorsIO = intakeSensorsIO;
    this.index = index;
  }

  public void updateInputs() {
    intakeSensorsIO.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeSensors" + index, inputs);
    // FIXME: Change this directory so it's actually correct
  }

  // if the object is detected and within a certain distance then return true
  public boolean isReadyToIntake() {
    if (getIsDetected() == true
        && getDistance() < 0.5 /*FIXME: CHANGE THIS VALUE TO WHAT WE NEED*/) {
      return true;
    } else {
      return false;
    }
  }
  // FIXME: make it so that if this method returns true, then the robot will align to or intake the
  // piece

  // getters
  public double getDistance() {
    return inputs.distance;
  }

  public boolean getIsDetected() {
    return inputs.isDetected;
  }
}
