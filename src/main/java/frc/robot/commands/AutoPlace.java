package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class AutoPlace extends CommandBase {
    
    private Hand handMotor;
    private double speed;
    private Timer t;
    public AutoPlace(Hand h, double s){
        handMotor = h;
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
        handMotor.setHandSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        handMotor.setHandSpeed(0);
        t.stop();
    }
    @Override
    public boolean isFinished(){return t.get() > 3;}
}
