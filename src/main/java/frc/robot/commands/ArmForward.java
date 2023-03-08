package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ArmForward extends CommandBase {
    
    
    private Arm armMotor;
    
    public ArmForward(Arm a){
        armMotor = a;
    }
    public void initialize(){}

    public void execute(){
        armMotor.setMotor(0.3);
    }
    public void end(boolean interrupted){
        armMotor.setMotor(0);
        
    }
    public boolean isFinished(){return false;}
}
