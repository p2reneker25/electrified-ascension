package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTime extends CommandBase {
    DriveTrain drive;
    double time;
    Timer timer;

    public DriveTime(DriveTrain d, double t) {
        drive = d;
        t = time;  
        timer = new Timer();      
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        drive.autoMode = true;
        // currentdist = Math.abs(drive.frontleft.getDriveEncoder());
        drive.driveInDirection(-0.5, 0);
        System.out.println("DRIVETIME");
        // drive.chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
        
    }
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
    @Override
    public boolean isFinished() {return (timer.get() > 3);}
}
