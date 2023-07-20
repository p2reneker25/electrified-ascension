package frc.robot.autos;

import javax.print.attribute.standard.RequestingUserName;
//amogus
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPlace9001;
import frc.robot.commands.AutoZero;
import frc.robot.commands.Set9001;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Jackson9001;

public class TwoCube9001 extends SequentialCommandGroup {
    PathPlannerTrajectory grabTrajectory = PathPlanner.loadPath("GrabPath", new PathConstraints(4.88, 4));
    PathPlannerTrajectory returnTrajectory = PathPlanner.loadPath("ReturnPath", new PathConstraints(4.88, 4));
    //shoots init cube, grabs cube and bring it back to shoot
    public TwoCube9001(DriveTrain drive, Jackson9001 intake){
        addCommands(
            //places cube high
           new AutoPlace9001(intake, 1),
           //follows trajectory to pick up cube while lowering intake
            new ParallelCommandGroup(drive.followTrajectoryCommand(grabTrajectory, true), new Set9001(intake,true)),
            //intakes the cube (wow really?)
            new AutoIntake(intake),
            //follows trajectory back to score while lifting intake 
            new ParallelCommandGroup(drive.followTrajectoryCommand(returnTrajectory, false), new Set9001(intake,false)),
            //places in low
            new AutoPlace9001(intake, 0.25),
            //stops just in case robot hasnt stopped for some reason
            new AutoZero(drive)
        );
    }
}
