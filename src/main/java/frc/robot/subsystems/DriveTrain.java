// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final DriveModule frontright;
  private final DriveModule frontleft;
  private final DriveModule backright;
  private final DriveModule backleft;
  private ChassisSpeeds chassisSpeeds;
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DriveConstants.CHASSIS_LENGTH / 2,Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(Constants.DriveConstants.CHASSIS_LENGTH,-Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(-Constants.DriveConstants.CHASSIS_LENGTH / 2,Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(-Constants.DriveConstants.CHASSIS_LENGTH/2,-Constants.DriveConstants.CHASSIS_WIDTH/2)
  );

  public DriveTrain() {
    frontright = new DriveModule(
      Constants.DriveConstants.FRONTRIGHT_MODULE_DRIVE_CAN,
      Constants.DriveConstants.FRONTRIGHT_MODULE_STEER_CAN,
      Constants.DriveConstants.FRONTRIGHT_MODULE_ENCODER,
      Constants.DriveConstants.FRONTRIGHT_MODULE_OFFSET
    );
    frontleft = new DriveModule(
      Constants.DriveConstants.FRONTLEFT_MODULE_DRIVE_CAN,
      Constants.DriveConstants.FRONTLEFT_MODULE_STEER_CAN,
      Constants.DriveConstants.FRONTLEFT_MODULE_ENCODER,
      Constants.DriveConstants.FRONTLEFT_MODULE_OFFSET
    );
    backright = new DriveModule(
      Constants.DriveConstants.BACKRIGHT_MODULE_DRIVE_CAN,
      Constants.DriveConstants.BACKRIGHT_MODULE_STEER_CAN,
      Constants.DriveConstants.BACKRIGHT_MODULE_ENCODER,
      Constants.DriveConstants.BACKRIGHT_MODULE_OFFSET
    );
    backleft = new DriveModule(
      Constants.DriveConstants.BACKLEFT_MODULE_DRIVE_CAN,
      Constants.DriveConstants.BACKLEFT_MODULE_STEER_CAN,
      Constants.DriveConstants.BACKLEFT_MODULE_ENCODER,
      Constants.DriveConstants.BACKLEFT_MODULE_OFFSET
    );
  
      chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }
  public void drive(double x, double y) {
    if (Math.abs(x) < 0.12) {x = 0;}
    if (Math.abs(y) < 0.12) {y = 0;}
    chassisSpeeds.vxMetersPerSecond = -y;
    chassisSpeeds.vyMetersPerSecond = -x;
  }
  @Override
  public void periodic() {
    SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);

    SmartDashboard.putNumber("frontright speed: ", state[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontright angle: ", state[0].angle.getDegrees() + frontright.offset*360);
    SmartDashboard.putNumber("frontright encoder: ", frontright.getEncoder());
    SmartDashboard.putNumber("frontright wrapoffset: ", frontright.wrapoffset);

    SmartDashboard.putNumber("frontleft speed: ", state[1].speedMetersPerSecond);
    SmartDashboard.putNumber("frontleft angle: ", state[1].angle.getDegrees() + frontleft.offset*360);
    SmartDashboard.putNumber("frontleft encoder: ", frontleft.getEncoder());
    SmartDashboard.putNumber("frontleft wrapoffset: ", frontleft.wrapoffset);

    SmartDashboard.putNumber("backright speed: ", state[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backright angle: ", state[2].angle.getDegrees() + backright.offset*360);
    SmartDashboard.putNumber("backright encoder: ", backright.getEncoder());
    SmartDashboard.putNumber("backright wrapoffset: ", backright.wrapoffset);

    SmartDashboard.putNumber("backleft speed: ", state[3].speedMetersPerSecond);
    SmartDashboard.putNumber("backleft angle: ", state[3].angle.getDegrees() + backleft.offset*360);
    SmartDashboard.putNumber("backleft encoder: ", backleft.getEncoder());
    SmartDashboard.putNumber("backleft wrapoffset: ", backleft.wrapoffset);

    frontright.set(state[0].speedMetersPerSecond, state[0].angle.getDegrees());
    frontleft.set(state[1].speedMetersPerSecond, state[1].angle.getDegrees());
    backright.set(state[2].speedMetersPerSecond, state[2].angle.getDegrees());
    backleft.set(state[3].speedMetersPerSecond, state[3].angle.getDegrees());
    //t.set(0.5);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
