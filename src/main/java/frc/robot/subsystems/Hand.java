package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class Hand {
   private VictorSP handMotor; 
    public Hand(){
        handMotor = new VictorSP(Constants.HandConstants.HAND_MOTOR_PWM);
    }
    public void setHandSpeed(double speed){
        handMotor.set(speed);
    }
}
