package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Jackson9001;

public class AutoIntake extends CommandBase {
    
    private Jackson9001 intake;
    
    private Timer t;
    public AutoIntake(Jackson9001 j){
        intake = j;
    }
    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        intake.setRollerSpeed(-1);
    }
    @Override
    public void end(boolean interrupted){
        intake.setRollerSpeed(0);
    }
    @Override
    public boolean isFinished(){return intake.getAmps() > 50;}
}
