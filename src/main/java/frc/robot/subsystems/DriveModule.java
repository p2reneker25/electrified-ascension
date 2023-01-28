package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;

public class DriveModule {
    private final CANSparkMax drive;
    private final CANSparkMax steer;
    private final DutyCycleEncoder encoder;
    public final double offset;
    private double lastoffset;
    public double wrapoffset = 0;
    public boolean firstwrap = true;
    public boolean stopped = false;
    public double flip = 0;
    public boolean flipBool = false;
    PIDController pid;
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
        pid = new PIDController(DriveConstants.P, DriveConstants.I, DriveConstants.D);
        //pid.setTolerance(16);
        // pid.setTolerance(16, 1);
        pid.enableContinuousInput(0, 360);
    }
    public void set(double drivespeed, double angle) {
        // pid.setP(SmartDashboard.getNumber("P", DriveConstants.P));
        // pid.setI(SmartDashboard.getNumber("I", DriveConstants.I));
        // pid.setD(SmartDashboard.getNumber("D", DriveConstants.D));
        angle += offset * 360.0;
        // if (pid.getPositionError() > 135) {
        //     flip+=1;
        //     if (flipBool) {flipBool = false; } else {flipBool = }
        // }
        double mspeed = MathUtil.clamp(-pid.calculate(getEncoder(), angle), -0.2, 0.2);
        
        
        // if (mspeed > 0.15) {
        //     mspeed = 0.15;
        // }
        // if (Math.abs(getEncoder()-angle) > 4) {
        //     if (getEncoder() < angle) {steer.set(-0.1);}
        //     else if (getEncoder() > angle) {steer.set(0.1);}
        // }else {
        //     steer.set(0.0);
        // }
        // if (!pid.atSetpoint()) {
        //     steer.set(pid.calculate(getEncoder(), angle));
        // }else {
        //     steer.set(0);
        // }
        if (pid.atSetpoint() == false) {
            steer.set(mspeed);        
        }else {
           // System.out.println("at setpoint");
            steer.set(0);
        }
        
        
        drive.set(drivespeed * 0.5);
    }
    public double getEncoder() {
        // if (firstwrap) {
        //     double enc = encoder.getAbsolutePosition();
        //     lastoffset = enc;
        //     firstwrap = false;
        //     return enc;
        // }
        // double enc = encoder.getAbsolutePosition();
        // if (Math.abs(enc-lastoffset) > 0.5) {
        //     System.out.println("wrapped: " + (enc-lastoffset) + " enc: " + enc + " lastoffset: " + lastoffset);
        //     wrapoffset -= (enc-lastoffset);
        // }
        // lastoffset = enc;
        return (encoder.getAbsolutePosition()) * 360.0;
    }
}
