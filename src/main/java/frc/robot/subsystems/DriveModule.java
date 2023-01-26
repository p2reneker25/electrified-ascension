package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DriveModule {
    public final CANSparkMax drive;
    public final CANSparkMax steer;
    public final DutyCycleEncoder encoder;
    public DriveModule(
        int DRIVE_CAN,
        int STEER_CAN,
        int ENCODER_DIO,
        double OFFSET
    ) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        steer = new CANSparkMax(STEER_CAN, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(new DigitalInput(ENCODER_DIO));
        encoder.setPositionOffset(OFFSET);
    }
    public void set(double drivespeed, double angle) {
        drive.set(drivespeed * 0.5);
    }
    public double getEncoder() {
        return encoder.getAbsolutePosition();
    }
}
