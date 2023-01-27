package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DriveModule {
    private final CANSparkMax drive;
    private final CANSparkMax steer;
    private final DutyCycleEncoder encoder;
    public final double offset;
    private double lastoffset;
    public double wrapoffset = 0;
    public boolean firstwrap = true;
    public DriveModule(
        int DRIVE_CAN,
        int STEER_CAN,
        int ENCODER_DIO,
        double OFFSET
    ) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        steer = new CANSparkMax(STEER_CAN, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(new DigitalInput(ENCODER_DIO));
        offset = OFFSET;
        lastoffset = encoder.getAbsolutePosition();
        wrapoffset = 0;
    }
    public void set(double drivespeed, double angle) {
        double dspeed = 0;
        angle += offset * 360.0;
        if (Math.abs(getEncoder()-angle) > 8) {
            if (getEncoder() < angle) {steer.set(-0.15);}
            else if (getEncoder() > angle) {steer.set(0.15);}
        }else {
            dspeed = (drivespeed * 0.5);
            steer.set(0.0);
        }
        drive.set(drivespeed * 0.5);
    }
    public double getEncoder() {
        if (firstwrap) {
            double enc = encoder.getAbsolutePosition();
            lastoffset = enc;
            firstwrap = false;
            return enc;
        }
        double enc = encoder.getAbsolutePosition();
        if (Math.abs(enc-lastoffset) > 0.5) {
            System.out.println("wrapped: " + (enc-lastoffset) + " enc: " + enc + " lastoffset: " + lastoffset);
            wrapoffset -= (enc-lastoffset);
        }
        lastoffset = enc;
        return (encoder.getAbsolutePosition() + wrapoffset) * 360.0;
    }
}
