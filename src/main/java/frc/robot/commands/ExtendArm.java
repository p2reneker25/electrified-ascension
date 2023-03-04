package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArm extends CommandBase{
    private Arm arm;
    private double speed;
    public double initialPos = 0;
    public boolean stop = false;
    public ExtendArm(Arm ar, double s) {
        arm = ar;
        speed = s;
    }
    @Override
    public void initialize() {
        initialPos = arm.getEncoder();
        stop = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setExtendSpeed(speed);
        if (stop == false) {
            if (Math.abs(arm.getEncoder()-initialPos) > 5.0) {
                stop = true;
            }
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setExtendSpeed(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
