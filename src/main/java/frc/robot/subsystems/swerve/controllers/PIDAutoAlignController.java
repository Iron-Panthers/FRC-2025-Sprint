package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import java.util.function.Supplier;

public class PIDAutoAlignController {

  // supplies the position values
  private ProfiledPIDController magController;
  private Supplier<Pose2d> positionSupplier;

  // target position
  private Pose2d targetPosition;
  private Pose2d startPosition;
  private double xVel;
  private double yVel;
  private final Supplier<Rotation2d> yawSupplier;

  public PIDAutoAlignController(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {

    this.positionSupplier = positionSupplier;
    this.yawSupplier = yawSupplier;
    this.targetPosition = targetPosition;
    setTargetPosition(targetPosition);

    // setting up the ProfiledPIDController
    magController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
  }
  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    // the naming is very important
    double dy = targetPosition.getY() - positionSupplier.get().getY();
    double dx = targetPosition.getX() - positionSupplier.get().getX();
    double angle = Math.atan2(dy, dx);
    double magStartPos = Math.hypot(startPosition.getX(), startPosition.getY());
    double translMagCurrPos =
        Math.hypot(positionSupplier.get().getX(), positionSupplier.get().getY()) - magStartPos;
    double translMagTargPos =
        Math.hypot(targetPosition.getX(), targetPosition.getY()) - magStartPos;
    double magVel = magController.calculate(translMagCurrPos, translMagTargPos);
    yVel = Math.abs(magVel * Math.sin(angle)) * (dy < 0 ? -1 : 1);
    xVel = Math.abs(magVel * Math.cos(angle)) * (dx < 0 ? -1 : 1);
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-xVel, -yVel, 0, yawSupplier.get());
  }
  // log your data in advantage kit
  public Pose2d getTargetPosition() {
    return targetPosition;
  }

  public double getXVel() {
    return -xVel;
  }

  public double getYVel() {
    return -yVel;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
  }
}
