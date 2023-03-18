package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class HandCylinder extends CommandBase {
    private Hand hand;
    private Timer timer;
    private Value value;
    public HandCylinder(Hand h,Value v){
        hand = h;
        timer = new Timer();
        value = v;
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        hand.setHandCylinder(value);
    }
    @Override
    public void end(boolean interrupted){
        hand.setHandCylinder(Value.kOff);
        timer.stop();
       
        
    }
    @Override
    public boolean isFinished(){return timer.get() > 1;}
}
