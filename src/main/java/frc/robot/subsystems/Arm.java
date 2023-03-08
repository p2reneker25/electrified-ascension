package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;


public class Arm {
    private TalonFX talon;
    private MotorController armMotor;
    public Arm(){
        armMotor = new VictorSP(Constants.ArmConstants.ARM_PWM);
        talon = new TalonFX(Constants.ArmConstants.PIVOT_CAN);
    }
    public void setMotor(double speed){
        armMotor.set(speed);
    }
    public void setPivot(double speed){
        talon.set(TalonFXControlMode.PercentOutput, speed);
    }

    
}    
