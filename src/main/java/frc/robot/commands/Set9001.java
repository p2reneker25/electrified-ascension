package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Jackson9001;

public class Set9001 extends CommandBase{
    private Jackson9001 j;
    private Timer t;
    private boolean pos;
    
    public Set9001(Jackson9001 jackson9001, boolean pos){
        j=jackson9001;
        t = new Timer();
        this.pos=pos;
    }
    @Override
    public void initialize(){
        t.reset();
        t.start();
    }
    @Override
    public void execute(){
        if(pos)
            j.setRollerCylinder(Value.kForward);
        else
            j.setRollerCylinder(Value.kReverse);
    }
    @Override
    public void end(boolean interrupted){
        t.stop();
        t.reset();
        j.setRollerCylinder(Value.kOff);

    }
    @Override
    public boolean isFinished(){
        return t.get() > 0.75;

    }
    
}
