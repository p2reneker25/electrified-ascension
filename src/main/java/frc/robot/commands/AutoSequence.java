package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hand;

public class AutoSequence extends SequentialCommandGroup {
    public AutoSequence(DriveTrain drive, Hand hand) {
        addCommands(
            new AutoPlace(hand, 1)
            // new DriveAuto(drive, 0, 2.4, 0.5)
            // new AutoBalence(drive)
 
        );
    }
}
