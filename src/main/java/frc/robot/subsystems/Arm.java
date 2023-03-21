package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{
    private TalonFX talon;
    CANSparkMax extendMotor;
    DoubleSolenoid brake;
    double offset = 0;
    Timer timer = new Timer();
    boolean brakeCylenderEngaged = false;
    public Arm(){
        extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
        brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.BRAKE_1, ArmConstants.BRAKE_2);
        offset = getEncoder();
        talon = new TalonFX(Constants.ArmConstants.PIVOT_CAN);
        talon.setNeutralMode(NeutralMode.Brake);
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
    public void setBrake(Value v) {
        brake.set(v);
    }
    public void engageBrake(Value v) {
        setBrake(v);

        brakeCylenderEngaged = true;
        timer.reset();
        timer.start();
    }
    @Override
    public void periodic() {
        if (timer.get() > 0.5) {
            timer.stop();
            brakeCylenderEngaged = false;
            setBrake(Value.kOff);
        }
        SmartDashboard.putNumber("ARM ENCODER", getEncoder());
    }
}

