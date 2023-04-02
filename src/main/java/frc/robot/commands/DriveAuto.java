package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
    DriveTrain drive;
    double currentdist = 0;
    double initValue = 0;
    double angle;
    double distance;
    double speed;

    public DriveAuto(DriveTrain d, double angle, double distance, double speed) {
        drive = d;
        this.angle = angle;
        this.speed = speed;
        this.distance = distance;
    }
    @Override
    public void initialize() {
        currentdist = 0;
        initValue = drive.getDriveEncoderAvg();
    }
    @Override
    public void execute() {
        drive.autoMode = true;
        currentdist = drive.getDriveEncoderAvg()-initValue;
        drive.driveInDirection(-speed, angle);
    }
    @Override
    public void end(boolean interrupted) {
        drive.autoMode = false;
    }
    @Override
    public boolean isFinished() {return (currentdist < -distance);}
}
