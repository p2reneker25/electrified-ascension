package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ResetToStart extends CommandBase {
    boolean reset = false;
    DriveTrain drive;
    public ResetToStart(DriveTrain d) {
        drive = d;
    }
    @Override
    public void initialize(){
        reset = false;
    }
    @Override
    public void execute(){
        drive.NAVX.reset();
        drive.frontleft.resetDriveEncoder();
        drive.frontright.resetDriveEncoder();
        drive.backleft.resetDriveEncoder();
        drive.backright.resetDriveEncoder();
        reset = true;
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){return reset;}
}
