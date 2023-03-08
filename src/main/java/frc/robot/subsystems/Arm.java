package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{
    private TalonFX talon;
    CANSparkMax extendMotor;
    double offset = 0;
    public Arm(){
        extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
        offset = getEncoder();
        talon = new TalonFX(Constants.ArmConstants.PIVOT_CAN);
    }
    public void setExtendSpeed(double speed){
        extendMotor.set(speed);
    }
    public void setPivotSpeed(double speed){
        talon.set(TalonFXControlMode.PercentOutput, speed);
    }
    public double getEncoder() {
        return extendMotor.getEncoder().getPosition() - offset;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM ENCODER", getEncoder());
    }
}

