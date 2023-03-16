package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{
    CANSparkMax extendMotor;
    DoubleSolenoid brake;
    double offset = 0;
    public Arm(){
        extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
        brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.BRAKE_1, ArmConstants.BRAKE_2);
        offset = getEncoder();
    }
    public void setExtendSpeed(double speed){
        extendMotor.set(speed);
    }
    public double getEncoder() {
        return extendMotor.getEncoder().getPosition() - offset;
    }
    public void setBrake(Value v) {
        brake.set(v);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM ENCODER", getEncoder());
    }
}
