package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class HandCylinder extends CommandBase {
    private Hand hand;
    private Timer timer;
    private boolean pos = false;
    public HandCylinder(Hand h){
        hand = h;
        timer = new Timer();
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        pos = !pos;
        if (pos) {
            hand.setHandCylinder(Value.kReverse);
            // System.out.println("claw open");
        }else {
            // System.out.println("claw close");
            hand.setHandCylinder(Value.kForward);
        }
        timer.reset();
        timer.start();
        
    }
    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted){
        hand.setHandCylinder(Value.kOff);
        timer.stop();
       
        
    }
    @Override
    public boolean isFinished(){return timer.get() > 1;}
}
