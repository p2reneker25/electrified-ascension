package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class AutoBalance extends CommandBase {
    
    
    private DriveTrain drive;
    PIDController pid;
    double lastSensor = 0;
    int alternate_count = 0;
    double lastSpeed = 0;
    public AutoBalance(DriveTrain d){
        drive = d;
        pid = new PIDController(0.02, 0, 0);
    }
    public void initialize(){
        alternate_count = 0;
    }

    public void execute(){
        drive.autoMode = true;
        double gyro = drive.NAVX.getPitch() + 8.71;
        gyro += lastSensor;
        gyro /= 2;
        double pidValue = pid.calculate(gyro, 0);
        if (pidValue > 0.5) {pidValue = 0.5;}
        if (pidValue < -0.5) {pidValue = -0.5;}
        // drive.driveAuto(0, pidValue, 0);
        
        drive.frontright.set(-pidValue, 0, true);
        drive.frontleft.set(pidValue, 0, false);
        drive.backright.set(-pidValue, 0, false);
        drive.backleft.set(-pidValue, 0, true);
        
    }
    public void end(boolean interrupted){
        drive.autoMode = false;
    }
    public boolean isFinished(){return false;}
}
