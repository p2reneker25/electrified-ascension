package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Jackson9001;

public class MoveIntake extends CommandBase{
    public Jackson9001 roller;
    public Value value;
    public MoveIntake(Jackson9001 r, Value v){
        roller = r;
        value = v;

    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        roller.setRollerCylinder(value);
    }
    @Override
    public void end(boolean interrupted){
        roller.setRollerSpeed(0);
        
    }
    @Override
    public boolean isFinished(){return false;}
}

