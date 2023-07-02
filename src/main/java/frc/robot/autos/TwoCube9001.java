package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Jackson9001;

public class TwoCube9001 extends SequentialCommandGroup {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("2CubeBalance", new PathConstraints(4.88, 4));

    public TwoCube9001(DriveTrain drive, Jackson9001 intake){
        addCommands(
           
        );
    }
}
