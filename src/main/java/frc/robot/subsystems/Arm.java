package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
public class Arm extends SubsystemBase{
    CANSparkMax extendMotor;
    
    public Arm(){
       extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
    }
    public void setExtendSpeed(double speed){
        extendMotor.set(speed);
    }
    public double getEncoder() {
        return extendMotor.getEncoder().getPosition();
    }
}
