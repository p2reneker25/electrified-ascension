package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
public class Wrist extends SubsystemBase {
    CANSparkMax speedcontroller;
    int targetAngle;
    public Wrist() {
        speedcontroller = new CANSparkMax(ClawConstants.WRIST_CAN, MotorType.kBrushless);
    }
    public void rotateDegrees(int degrees) {
        targetAngle += degrees;
    }
    @Override
    public void periodic() {
        RelativeEncoder builtin = speedcontroller.getEncoder();
        double pos = builtin.getPosition();
        double targetpos = (targetAngle/360.0);
        if (Math.abs(pos-targetpos) > 0.04) {
            if (pos < targetpos) {
                speedcontroller.set(0.02);
            }else {
                speedcontroller.set(-0.02);
            }
        }else {
            speedcontroller.set(0);
        }
        SmartDashboard.putNumber("Wrist Encoder Value", pos);
        SmartDashboard.putNumber("Wrist Target Value", (targetAngle/360.0));

    }
}
