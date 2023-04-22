package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hand;

public class AutoSequence extends SequentialCommandGroup {
    public AutoSequence(DriveTrain drive, Hand hand) {
        addCommands(
            new ResetToStart(drive),
            new AutoPlace(hand, -0.5),
            new DriveTime(drive, 3),
            // new DriveAutoPlatform(drive, 0,  150*(0.7091), 0.5),
            // new DriveAutoPlatform2(drive, 0,  150*(0.7091), 0.1),
            new AutoZero(drive)
 
        );
    }
}
