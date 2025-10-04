package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.subsystems.swerve.DriveConstants;

public class RobotSimState {

    // SIMULATION VARIABLES
    private SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());

    public SwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    // SINGLETON IMPLEMENTATION
    private static RobotSimState instance = null;

    public static RobotSimState getInstance() {
        if (instance == null) {
            instance = new RobotSimState();
        }
        return instance;
    }

}
