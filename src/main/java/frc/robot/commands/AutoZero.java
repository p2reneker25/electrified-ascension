package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoZero extends CommandBase {
    DriveTrain drive;

    public AutoZero(DriveTrain d) {
        drive = d;

    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        drive.autoMode = true;
        // drive.chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
        drive.driveInDirection(0, 0);
        System.out.println("AUTOZERO");
    }
    @Override
    public void end(boolean interrupted) {
        drive.autoMode = false;
    }
    @Override
    public boolean isFinished() {return false;}
}
