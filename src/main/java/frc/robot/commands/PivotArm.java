package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class PivotArm extends CommandBase {
    
    
    private Arm arm;
    private double speed;
    public PivotArm(Arm a,double s){
        arm = a;
        speed = s;
    }
    @Override
    public void initialize(){
        arm.engageBrake(Value.kForward);
    }
    @Override
    public void execute(){
        
        System.out.println("arm moving at speed:" + speed);
        // if (arm.getLimit() == true) {
        //     if (Constants.ArmConstants.LOW_GEAR) {
        //         arm.setPivotSpeed(-speed);
        //     }else {
        //         arm.setPivotSpeed(-speed);
        //     }
            
        // }else {
        //     arm.setPivotSpeed(0);
        //     arm.engageBrake(Value.kReverse);
        // }
        if (Constants.ArmConstants.LOW_GEAR) {
            arm.setPivotSpeed(-speed);
        }else {
            arm.setPivotSpeed(-speed);
        }
        
    }
    @Override
    public void end(boolean interrupted){
        arm.setPivotSpeed(0);
        arm.engageBrake(Value.kReverse);
    }
    @Override
    public boolean isFinished(){return false;}
// 
}
