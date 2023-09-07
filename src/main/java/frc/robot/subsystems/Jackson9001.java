package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Jackson9001 extends SubsystemBase {
    TalonFX rollerMotor;
    DoubleSolenoid solenoid;
    public Jackson9001(){
        rollerMotor = new TalonFX(12);
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }
    public void setRollerSpeed(double speed){
        rollerMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setRollerCylinder(Value v){
        solenoid.set(v);
    }
    public double getAmps(){
        return rollerMotor.getSupplyCurrent();
    }

}
