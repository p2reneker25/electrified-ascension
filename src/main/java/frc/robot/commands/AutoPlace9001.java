package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Jackson9001;

public class AutoPlace9001 extends CommandBase {
    
    private Jackson9001 intake;
    private double speed;
    private Timer t;
    public AutoPlace9001(Jackson9001 j, double s){
        intake = j;
        speed = s;
        t = new Timer();
    }
    @Override
    public void initialize(){
        t.reset();
        t.start();
    }
    @Override
    public void execute(){
        intake.setRollerSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        intake.setRollerSpeed(0);
        t.stop();
    }
    @Override
    public boolean isFinished(){return t.get() > 1.5;}
}
