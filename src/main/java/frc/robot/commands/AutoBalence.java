package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class AutoBalence extends CommandBase {
    
    
    private DriveTrain drive;
    PIDController pid;
    double lastSensor = 0;
    int alternate_count = 0;
    double lastSpeed = 0;
    public AutoBalence(DriveTrain d){
        drive = d;
        pid = new PIDController(0.5, 0, 0);
    }
    public void initialize(){
        alternate_count = 0;
    }

    public void execute(){

        double accelerometer = drive.getAccelerometer().getX();
        accelerometer += lastSensor;
        accelerometer /= 2;
        double pidValue = pid.calculate(accelerometer, 0);
        if (pidValue > 0.5) {pidValue = 0.5;}
        if (pidValue < -0.5) {pidValue = -0.5;}
        // drive.driveAuto(0, pidValue, 0);
        pidValue = -pidValue;
        if ((lastSpeed > 0 && pidValue < 0) || (lastSpeed < 0 && pidValue > 0)) {
            alternate_count++;
        }
        drive.frontright.set(-pidValue, 0, true);
        drive.frontleft.set(pidValue, 0, false);
        drive.backright.set(-pidValue, 0, false);
        drive.backleft.set(-pidValue, 0, true);
        lastSensor = drive.getAccelerometer().getX();
        drive.autoMode = true;
    }
    public void end(boolean interrupted){
        drive.autoMode = false;
    }
    public boolean isFinished(){return alternate_count > 5;}
}
