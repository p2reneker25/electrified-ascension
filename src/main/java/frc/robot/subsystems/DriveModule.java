package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.DriveConstants;

public class DriveModule {
    public final CANSparkMax drive;
    public final CANSparkMax steer;
    public final DutyCycleEncoder encoder;
    public final double offset;
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
        drive.setSmartCurrentLimit(25);
        encoder = new DutyCycleEncoder(new DigitalInput(ENCODER_DIO));
        offset = OFFSET;
          pid = new PIDController(DriveConstants.P, DriveConstants.I, DriveConstants.D);
        steerOffset = getNEOEncoder();
        // drive.getEncoder().setPositionConversionFactor(0.7091);
        pid.enableContinuousInput(0, 360);
    }
    public void stop() {
        drive.set(0);
        steer.set(0);

    }
    public void resetDriveEncoder() {
        initDriveEncoder = drive.getEncoder().getPosition();

    }
    public double getNEOEncoder() {
        return (steer.getEncoder().getPosition() * 60.0) - steerOffset;
    }
    public void set(double drivespeed, double angle, boolean flipPID) {
        angle += offset * 360.0;
        double flip = -1;
        if (flipPID == true) {flip = 1;}
        
        double mspeed = MathUtil.clamp(pid.calculate(getEncoder(), angle) * flip, -1, 1);
        
        double dif = pid.getPositionError();
        
        if (pid.atSetpoint() == false) {
            steer.set(mspeed);        
        }else {
            steer.set(0);
        }
        
        
        drive.set(drivespeed);
    }
    public void setDesiredState(SwerveModuleState state,boolean flipPID){
        set(state.speedMetersPerSecond,state.angle.getDegrees(),flipPID);
    }

    public double getDriveEncoder() {
        return drive.getEncoder().getPosition()-initDriveEncoder;
    }
    public double getEncoderDeviation(double avg) {
        return Math.abs(avg-getDriveEncoder());
    }
    public double getEncoder() {
        return (encoder.getAbsolutePosition() * 360.0);
    }
}
