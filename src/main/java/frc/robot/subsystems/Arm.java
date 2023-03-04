package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{
    CANSparkMax extendMotor;
    double offset = 0;
    public Arm(){
        extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
        offset = getEncoder();
    }
    public void setExtendSpeed(double speed){
        extendMotor.set(speed);
    }
    public double getEncoder() {
        return extendMotor.getEncoder().getPosition() - offset;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM ENCODER", getEncoder());
    }
}
