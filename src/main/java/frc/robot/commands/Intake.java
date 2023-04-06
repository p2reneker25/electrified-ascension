package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Jackson9001;

public class Intake extends CommandBase{
    public Jackson9001 roller;
    public double speed;
    public Intake(Jackson9001 r, double s){
        roller = r;
        speed = s;

    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        roller.setRollerSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        roller.setRollerSpeed(0);
        
    }
    @Override
    public boolean isFinished(){return false;}
}
