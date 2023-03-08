package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class PivotArm extends CommandBase {
    
    
    private Arm arm;
    private double speed;
    public PivotArm(Arm a,double s){
        arm = a;
        speed = s;
    }
    public void initialize(){}

    public void execute(){
        arm.setPivotSpeed(speed);
    }
    public void end(boolean interrupted){
        arm.setPivotSpeed(0);
        
    }
    public boolean isFinished(){return false;}
}
