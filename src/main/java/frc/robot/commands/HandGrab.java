package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class HandGrab extends CommandBase {
    
    private Hand handMotor;
    
    public HandGrab(Hand h){
        handMotor = h;
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        handMotor.setHandSpeed(0.5);
    }
    @Override
    public void end(boolean interrupted){
        handMotor.setHandSpeed(0);
        
    }
    @Override
    public boolean isFinished(){return false;}
}
