package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Jackson9001 extends SubsystemBase {
    TalonFX rollerMotor;
    public Jackson9001(){
        rollerMotor = new TalonFX(12);
    }
    public void setRollerSpeed(double speed){
        rollerMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
}
