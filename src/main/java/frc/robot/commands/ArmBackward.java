package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmBackward extends CommandBase{
    
    
    private Arm armMotor;
    
    public ArmBackward(Arm a){
        armMotor = a;
    }
    public void initialize(){}

    public void execute(){
        armMotor.setExtendSpeed(-0.3);
    }
    public void end(boolean interrupted){
        armMotor.setExtendSpeed(0);
        
    }
    public boolean isFinished(){return false;}
}
