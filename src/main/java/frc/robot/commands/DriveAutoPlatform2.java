package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveAutoPlatform2 extends CommandBase {
    DriveTrain drive;
    double currentdist = 0;
    double initValue = 0;
    double angle;
    double distance;
    double speed;

    public DriveAutoPlatform2(DriveTrain d, double angle, double distance, double speed) {
        drive = d;
        this.angle = angle;
        this.speed = speed;
        this.distance = distance;
        SmartDashboard.putNumber("currentdist", currentdist);

    }
    @Override
    public void initialize() {
        currentdist = 0;
        initValue = drive.getDriveEncoderAvg();
    }
    @Override
    public void execute() {
        drive.autoMode = true;
        currentdist = Math.abs(drive.frontleft.getDriveEncoder());
        drive.driveInDirection(-speed, 0);
        SmartDashboard.putNumber("currentdist", currentdist);
        // drive.chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
        
    }
    @Override
    public void end(boolean interrupted) {
        drive.autoMode = false;
        drive.driveInDirection(0, 0);
    }
    @Override
    public boolean isFinished() {return (Math.abs(drive.getPitch()) < 2);}
}
