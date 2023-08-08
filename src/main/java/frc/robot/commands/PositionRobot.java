package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class PositionRobot extends CommandBase {
    DriveTrain drive;
    Vision vision;
    private boolean over;
    Translation2d idealPos;
    private double dist; //distance from ideal pos
    private double speed = 0.2;
    private PathPlannerTrajectory traj;
    private Rotation2d targetDeg;
    private boolean first;
    public PositionRobot(DriveTrain d, Vision v, Translation2d ideal,double targetDegree, boolean isFirstPath) {
        drive = d;
        first = isFirstPath;
        vision = v;
        targetDeg = Rotation2d.fromDegrees(targetDegree);
        traj = PathPlanner.generatePath(TrajectoryConstants.TrajectoryConstraints,
        new PathPoint(drive.getPose().getTranslation(), drive.NAVX.getRotation2d()),
        new PathPoint(ideal, targetDeg)
        );
    }

    @Override
    public void initialize() {
        over = false;
    }

    @Override
    public void execute() {
        // drive.autoMode = true;
        // Translation2d botpose = vision.getRelativeBotPose().getTranslation();
        // double dx = botpose.getX() - idealPos.getX();
        // double dy = botpose.getY() - idealPos.getY();
        // dist = Math.sqrt(dx*dx + dy*dy);

        // double moveX = 0;
        // double moveY = 0;

        // moveX = dx / dist;
        // moveY = dy / dist;

        // if (Math.abs(dx) < 0.02) {
        //     moveX = 0;
        // }
        
        // if (Math.abs(dy) < 0.02) {
        //     moveY = 0;
        // }

        // drive.driveAuto(moveY * speed, moveX * speed, 0);
        drive.followTrajectoryCommand(traj, first);
        over = true;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveAuto(0, 0, 0);
        // drive.autoMode = false;

    }

    @Override
    public boolean isFinished() {
        //return dist < 0.02;
        return over;
    }
}
