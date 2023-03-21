package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Brake extends CommandBase{
    private Arm arm;
    private boolean brakeEngaged = false;
    private Timer t;
    public Brake(Arm ar) {
        arm = ar;
        t = new Timer();
    }
    @Override
    public void initialize() {
        brakeEngaged = !brakeEngaged;
        if (brakeEngaged) {
            arm.setBrake(Value.kForward);
            System.out.println("brake open");
        }else {
            System.out.println("brake close");
            arm.setBrake(Value.kReverse);
        }
        t.reset();
        t.start();
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setBrake(Value.kOff);
        t.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return t.get() > 0.5;
    }
}
