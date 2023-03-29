package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ArmSetGround extends CommandBase {
    
    
    private Arm arm;
    
    public ArmSetGround(Arm a){
        arm = a;
    }
    public void initialize(){
        arm.setBrake(Value.kForward);
    }

    public void execute(){
        // arm.(Arm.ArmPosition.GROUND);
    }
    public void end(boolean interrupted){
        arm.setPivotSpeed(0);
        arm.setBrake(Value.kReverse);
        arm.setExtendSpeed(0);
    }
    public boolean isFinished(){return false;}
}
