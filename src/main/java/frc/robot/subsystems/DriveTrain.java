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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SwerveModule frontright;
  private final SwerveModule frontleft;
  private final SwerveModule backright;
  private final SwerveModule backleft;
  private ChassisSpeeds chassisSpeeds;
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DriveConstants.CHASSIS_LENGTH / 2,Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(Constants.DriveConstants.CHASSIS_LENGTH,-Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(-Constants.DriveConstants.CHASSIS_LENGTH / 2,Constants.DriveConstants.CHASSIS_WIDTH / 2), 
      new Translation2d(-Constants.DriveConstants.CHASSIS_LENGTH/2,-Constants.DriveConstants.CHASSIS_WIDTH/2)
  );

  public DriveTrain() {
    frontright = Mk4SwerveModuleHelper.createNeo(
      Mk4SwerveModuleHelper.GearRatio.L1,
      DriveConstants.FRONTRIGHT_MODULE_DRIVE_CAN,
      DriveConstants.FRONTRIGHT_MODULE_STEER_CAN,
      DriveConstants.FRONTRIGHT_MODULE_STEER_ENCODER, 
      DriveConstants.FRONTRIGHT_MODULE_STEER_OFFSET);
      frontleft = Mk4SwerveModuleHelper.createNeo(
      Mk4SwerveModuleHelper.GearRatio.L1,
      DriveConstants.FRONTLEFT_MODULE_DRIVE_CAN,
      DriveConstants.FRONTLEFT_MODULE_STEER_CAN,
      DriveConstants.FRONTLEFT_MODULE_STEER_ENCODER, 
      DriveConstants.FRONTLEFT_MODULE_STEER_OFFSET);
      backright = Mk4SwerveModuleHelper.createNeo(
      Mk4SwerveModuleHelper.GearRatio.L1,
      DriveConstants.BACKRIGHT_MODULE_DRIVE_CAN,
      DriveConstants.BACKRIGHT_MODULE_STEER_CAN,
      DriveConstants.BACKRIGHT_MODULE_STEER_ENCODER, 
      DriveConstants.BACKRIGHT_MODULE_STEER_OFFSET);
      backleft = Mk4SwerveModuleHelper.createNeo(
      Mk4SwerveModuleHelper.GearRatio.L1,
      DriveConstants.BACKLEFT_MODULE_DRIVE_CAN,
      DriveConstants.BACKLEFT_MODULE_STEER_CAN,
      DriveConstants.BACKLEFT_MODULE_STEER_ENCODER, 
      DriveConstants.BACKLEFT_MODULE_STEER_OFFSET);
      chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }
  public void drive(double x, double y) {
    if (Math.abs(x) < 0.12) {x = 0;}
    if (Math.abs(y) < 0.12) {y = 0;}
    chassisSpeeds.vxMetersPerSecond = x;
    chassisSpeeds.vyMetersPerSecond = y;
  }
  @Override
  public void periodic() {
    SwerveModuleState state = kinematics.toSwerveModuleStates(chassisSpeeds)[0];
    frontright.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED, state.angle.getRadians());
    frontleft.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED, state.angle.getRadians());
    backright.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED, state.angle.getRadians());
    backleft.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED, state.angle.getRadians());
    // This method will be called once per scheduler run
    //t.set(0.5);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
