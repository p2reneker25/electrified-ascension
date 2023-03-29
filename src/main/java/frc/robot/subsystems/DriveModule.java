package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;

public class DriveModule {
    public final CANSparkMax drive;
    public final CANSparkMax steer;
    public final DutyCycleEncoder encoder;
    public final double offset;
    private double lastoffset;
    public double wrapoffset = 0;
    public boolean firstwrap = true;
    public boolean stopped = false;
    public double flip = 0;
    public boolean flipBool = false;
    public double steerOffset = 0;
    public PIDController pid;
    public double initDriveEncoder;
    public DriveModule(
        int DRIVE_CAN,
        int STEER_CAN,
        int ENCODER_DIO,
        double OFFSET
    ) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        initDriveEncoder = drive.getEncoder().getPosition();
        steer = new CANSparkMax(STEER_CAN, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(new DigitalInput(ENCODER_DIO));
        offset = OFFSET;
          pid = new PIDController(DriveConstants.P, DriveConstants.I, DriveConstants.D);
        //pid.setTolerance(16);
        // pid.setTolerance(16, 1);
        steerOffset = getNEOEncoder();
        pid.enableContinuousInput(0, 360);
    }
    public void stop() {
        drive.set(0);
        steer.set(0);

    }
    
    public double getNEOEncoder() {
        // System.out.println(steer.getEncoder().getCountsPerRevolution());
        return (steer.getEncoder().getPosition() * 60.0) - steerOffset;
    }
    public void set(double drivespeed, double angle, boolean flipPID) {
        // pid.setP(SmartDashboard.getNumber("P", DriveConstants.P));
        // pid.setI(SmartDashboard.getNumber("I", DriveConstants.I));
        // pid.setD(SmartDashboard.getNumber("D", DriveConstants.D));
        angle += offset * 360.0;
        // if (pid.getPositionError() > 135) {
        //     flip+=1;
        //     if (flipBool) {flipBool = false; } else {flipBool = }
        // }
        double flip = -1;
        if (flipPID == true) {flip = 1;}
        
        double mspeed = MathUtil.clamp(pid.calculate(getEncoder(), angle) * flip, -0.2, 0.2);
        
        double dif = pid.getPositionError();
        
        // if (dif > 90.0) {
        //     flipBool = !flipBool;
        //     offset += 180.0;
        //     System.out.println("Should Swap!");
        // }
        // if (flipBool) {
        //     drivespeed = -drivespeed;
        // }
        // mspeed = MathUtil.clamp(-pid.calculate(getEncoder(), angle), -0.2, 0.2);
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
        
        
        drive.set(drivespeed);
    }
    public double getDriveEncoder() {
        return drive.getEncoder().getVelocity();
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
        return (encoder.getAbsolutePosition() * 360.0);
    }
}
