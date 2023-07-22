package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoBuilder { //helper class to more efficiently use PathPlanner AutoBuilder
    Command fullAutoCommand;
    HashMap<String, Command> eventMap;
    ArrayList<PathPlannerTrajectory> pathGroup;
    public AutoBuilder(String pathName, String[] markers, Command[] Commands, DriveTrain drive){
        pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(
            TrajectoryConstants.MAX_TRAJECORY_VELOCITY_METERS_PER_SECOND,
            TrajectoryConstants.MAX_TRAJETORY_ACCELERATION_PETERS_PER_SECOND_SQUARED
            ));
        eventMap = new HashMap<>();
        for(int x = 0; x < markers.length; x++){
            eventMap.put(markers[x], Commands[x]);
        }
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drive::getPose,
         drive::resetPose, 
         new PIDConstants(DriveConstants.P, 0, 0),
         new PIDConstants(DriveConstants.P, 0, 0), 
         drive::setChassisSpeeds,
         eventMap,
         drive
        );
        fullAutoCommand = autoBuilder.fullAuto(pathGroup);
    }
    public Command getAutoCommand(){
        return fullAutoCommand;
    }
}

