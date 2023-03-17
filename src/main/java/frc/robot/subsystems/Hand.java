package frc.robot.subsystems;

import java.util.Timer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class Hand {
   private VictorSP handMotor; 
   private DoubleSolenoid handSolonoid;
   
    public Hand(){
        handMotor = new VictorSP(Constants.HandConstants.HAND_MOTOR_PWM);
        handSolonoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
         Constants.HandConstants.HAND_CYLINDER_UP,
          Constants.HandConstants.HAND_CYLINDER_DOWN);
    }
    public void setHandSpeed(double speed){
        handMotor.set(speed);
    }
    public void setHandCylinder(Value v){
        handSolonoid.set(v);
    }

}
