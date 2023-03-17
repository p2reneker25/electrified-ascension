package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class PositionRobot extends CommandBase {
    DriveTrain drive;
    Vision vision;

    Translation2d idealPos = new Translation2d(-0.3, 0);
    private double dist; //distance from ideal pos
    private double speed = 0.2;
    public PositionRobot(DriveTrain d, Vision v) {
        drive = d;
        vision = v;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drive.autoMode = true;
        Translation2d botpose = vision.getRelativeBotPose().getTranslation();
        double dx = botpose.getX() - idealPos.getX();
        double dy = botpose.getY() - idealPos.getY();
        dist = Math.sqrt(dx*dx + dy*dy);

        double moveX = 0;
        double moveY = 0;

        moveX = dx / dist;
        moveY = dy / dist;

        if (Math.abs(dx) < 0.02) {
            moveX = 0;
        }
        
        if (Math.abs(dy) < 0.02) {
            moveY = 0;
        }

        drive.driveAuto(moveY * speed, moveX * speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveAuto(0, 0, 0);
        drive.autoMode = false;
    }

    @Override
    public boolean isFinished() {
        return dist < 0.02;
    }
}
