package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{ //Subsystem no longer used
    private TalonFX talon;
    CANSparkMax extendMotor;
    DoubleSolenoid brake;
    double offset = 0;
    Timer timer = new Timer();
    DigitalInput limitswitch;
    boolean brakeCylenderEngaged = false;
    PIDController armSetPos;
    public enum ArmPosition {
        GROUND,
        HIGH
    }
    int highPos = -40000;
    int lowPos = -9500;
    public boolean atSetpoint = true;
    public Arm(){
        extendMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_CAN, MotorType.kBrushless);
        brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.BRAKE_1, ArmConstants.BRAKE_2);
        offset = getEncoder();
        talon = new TalonFX(Constants.ArmConstants.PIVOT_CAN);
        talon.setNeutralMode(NeutralMode.Brake);
        limitswitch = new DigitalInput(Constants.ArmConstants.LIMIT_SWITCH_DIO);
        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        armSetPos = new PIDController(0.001, 0, 0);
        armSetPos.setSetpoint(200);
    }
    public void setExtendSpeed(double speed){
        if (getEncoder() < -85.5 && speed < 0) {
            speed = 0;
        }
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
    public boolean getLimit() {
        return limitswitch.get();
    }
    public void engageBrake(Value v) {
        setBrake(v);

        brakeCylenderEngaged = true;
        timer.reset();
        timer.start();
    }
    public void goToPosition(ArmPosition p) {
        int target = 0;
        int targetExtension = 0;
        int encoderVal = (int)talon.getSelectedSensorPosition();
        boolean atPivot = false;
        boolean atExtend = false;
        if (p == ArmPosition.HIGH) {
            target = -40000;
            targetExtension = -85;
        }else if (p == ArmPosition.GROUND) {
            target = -20000;
            targetExtension = -42;
        }
        // setPivotSpeed(armSetPos.calculate(encoderVal, target));
        // if (encoderVal < target+1000) {
        //     setPivotSpeed(0.25);
        // }else if () {

        // }
        if ((int)getEncoder() > (int)targetExtension) {
            setExtendSpeed(0.3);
        }else if ((int)getEncoder() < (int)targetExtension) {
            setExtendSpeed(-0.3);
        }else {
            atExtend = true;
            setExtendSpeed(0);
        }
        if (atPivot) {
            setPivotSpeed(0);
            engageBrake(Value.kReverse);
        }
        if (atExtend && atSetpoint) {
            atSetpoint = true;
        }
    }
    @Override
    public void periodic() {
        if (timer.get() > 0.5) {
            timer.stop();
            brakeCylenderEngaged = false;
            setBrake(Value.kOff);
        }
        if (atSetpoint == false) {
            // goToPosition(ArmPosition.GROUND);
        }
        SmartDashboard.putNumber("ARM LENGTH ENCODER", getEncoder());
        SmartDashboard.putBoolean("Limit Switch", getLimit());
        SmartDashboard.putNumber("Arm Position", talon.getSelectedSensorPosition());
    }
}

