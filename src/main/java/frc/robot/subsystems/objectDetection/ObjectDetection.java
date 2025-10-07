package frc.robot.subsystems.objectDetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase{
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    private double xErr;
    private double yErr;
    //make a velocity algorithm (maybe)

    public ObjectDetection(){}


    @Override
    public void periodic() {
        xErr = table.getEntry("tx").getDouble(0);
        yErr = table.getEntry("tx").getDouble(0);
    }
}