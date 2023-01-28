package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;
public class Arm {
    private MotorController armMotor;
    public Arm(){
        armMotor = new VictorSP(Constants.ArmConstants.ARM_PWM);
    }
    public void setMotor(double speed){
        armMotor.set(speed);
    }

    
}    
