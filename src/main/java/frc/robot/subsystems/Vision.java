package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable limelight = inst.getTable("limelight");
    
    public Vision() {

    }

    @Override
    public void periodic() {
        Transform2d botpose = getBotPose();
        SmartDashboard.putNumber("robot x", botpose.getX());
        SmartDashboard.putNumber("robot y", botpose.getY());
        SmartDashboard.putNumber("robot angle", botpose.getRotation().getDegrees());
    }  
    public Transform2d getBotPose() {
        Transform2d t;
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[6]);
        t = new Transform2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(botpose[4]));
        return t;
    }
}
