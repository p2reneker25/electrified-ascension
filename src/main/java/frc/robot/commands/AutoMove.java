package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoMove extends CommandBase{
    
    
    private DriveTrain drive;
    
    public AutoMove(DriveTrain a){
        drive = a;
    }
    public void initialize(){

    }

    public void execute(){
        
    }
    public void end(boolean interrupted){
        // armMotor.setExtendSpeed(0);
        
    }
    public boolean isFinished(){return false;}
}
