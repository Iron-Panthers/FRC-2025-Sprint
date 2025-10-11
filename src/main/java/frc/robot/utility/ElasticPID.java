package frc.robot.utility;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElasticPID {
  SetSlot0Lambda lambda;

  GravityTypeValue gravityTypeValue;

  String folderName;

  // Default constructor uses 0 for PIDGains and MotionMagicConfig when neither are provided.
  public ElasticPID(
      SetSlot0Lambda setSlot0Function, GravityTypeValue gravityTypeValue, String folderName) {
    this(
        setSlot0Function,
        gravityTypeValue,
        folderName,
        new PIDGains(0, 0, 0, 0, 0, 0, 0),
        new MotionMagicConfig(0, 0, 0));
  }

  // Constructor should take in lambda for setSlot0 function
  public ElasticPID(
      SetSlot0Lambda setSlot0Function,
      GravityTypeValue gravityTypeValue,
      String folderName,
      PIDGains pidGains,
      MotionMagicConfig motionMagicConfig) {
    SmartDashboard.putNumber(folderName + "/kP", pidGains.kP());
    SmartDashboard.putNumber(folderName + "/kI", pidGains.kI());
    SmartDashboard.putNumber(folderName + "/kD", pidGains.kD());
    SmartDashboard.putNumber(folderName + "/kS", pidGains.kS());
    SmartDashboard.putNumber(folderName + "/kV", pidGains.kV());
    SmartDashboard.putNumber(folderName + "/kA", pidGains.kA());
    SmartDashboard.putNumber(folderName + "/kG", pidGains.kG());
    SmartDashboard.putNumber(
        folderName + "/motionMagicAcceleration", motionMagicConfig.acceleration());
    SmartDashboard.putNumber(
        folderName + "/motionMagicCruiseVelocity", motionMagicConfig.cruiseVelocity());
    SmartDashboard.putNumber(folderName + "/motionMagicJerk", motionMagicConfig.jerk());

    this.folderName = folderName;
    this.gravityTypeValue = gravityTypeValue;
    this.lambda = setSlot0Function;
  }

  // Periodic function
  public void periodic() {
    this.lambda.setSlot0(
        SmartDashboard.getNumber(folderName + "/kP", 0),
        SmartDashboard.getNumber(folderName + "/kI", 0),
        SmartDashboard.getNumber(folderName + "/kD", 0),
        SmartDashboard.getNumber(folderName + "/kS", 0),
        SmartDashboard.getNumber(folderName + "/kV", 0),
        SmartDashboard.getNumber(folderName + "/kA", 0),
        SmartDashboard.getNumber(folderName + "/kG", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicAcceleration", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicCruiseVelocity", 0),
        SmartDashboard.getNumber(folderName + "/motionMagicJerk", 0),
        gravityTypeValue);
  }

  // Interface for lambda
  public interface SetSlot0Lambda {
    public void setSlot0(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        double motionMagicAcceleration,
        double motionMagicCruiseVelocity,
        double motionMagicJerk,
        GravityTypeValue gravityTypeValue);
  }
}
