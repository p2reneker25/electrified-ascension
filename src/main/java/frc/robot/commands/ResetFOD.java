package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ResetFOD extends CommandBase {
    DriveTrain drive;
    public ResetFOD(DriveTrain d) {
        drive = d;
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        drive.resetFOD();
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){return false;}

}
