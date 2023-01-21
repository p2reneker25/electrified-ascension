package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class TurnWrist extends CommandBase {
    private Wrist wrist;
    public TurnWrist(Wrist wr) {
        wrist = wr;
    }
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wrist.rotateDegrees(90);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
}
