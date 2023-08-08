// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
public class DriveTrain extends SubsystemBase {
  
  public final DriveModule frontright;
  public final DriveModule frontleft;
  public final DriveModule backright;
  public final DriveModule backleft;
  public ChassisSpeeds chassisSpeeds;

  public boolean autoMode = false;
  public int hatValue = 0;
  public AHRS NAVX = new AHRS(I2C.Port.kOnboard);
  public Pose2d pose;
  private double navxoffset = 0;
  public PIDController navxpid;
  public Accelerometer accelerometer = new BuiltInAccelerometer();
  double drive_magnitude = 0;
  double navx_tilt = 0;
  double fodoffset = 0;
  public double pitchoffset = 0;
  
  
  //maps where the swerve modules are on the robot
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(-DriveConstants.CHASSIS_WIDTH / 2,DriveConstants.CHASSIS_LENGTH / 2), 
      new Translation2d(-DriveConstants.CHASSIS_WIDTH/2,-DriveConstants.CHASSIS_LENGTH/2),
      new Translation2d(DriveConstants.CHASSIS_WIDTH / 2,DriveConstants.CHASSIS_LENGTH / 2), 
      new Translation2d(DriveConstants.CHASSIS_WIDTH,-DriveConstants.CHASSIS_LENGTH / 2)
  );
  SwerveDriveOdometry m_odometry;
  public DriveTrain() {

    //initializing each module based on constants, explained in DriveModule
    frontright = new DriveModule(
      DriveConstants.FRONTRIGHT_MODULE_DRIVE_CAN,
      DriveConstants.FRONTRIGHT_MODULE_STEER_CAN,
      DriveConstants.FRONTRIGHT_MODULE_ENCODER,
      DriveConstants.FRONTRIGHT_MODULE_OFFSET
    );
    frontleft = new DriveModule(
      DriveConstants.FRONTLEFT_MODULE_DRIVE_CAN,
      DriveConstants.FRONTLEFT_MODULE_STEER_CAN,
      DriveConstants.FRONTLEFT_MODULE_ENCODER,
      DriveConstants.FRONTLEFT_MODULE_OFFSET
    );
    backright = new DriveModule(
      DriveConstants.BACKRIGHT_MODULE_DRIVE_CAN,
      DriveConstants.BACKRIGHT_MODULE_STEER_CAN,
      DriveConstants.BACKRIGHT_MODULE_ENCODER,
      DriveConstants.BACKRIGHT_MODULE_OFFSET
    );
    backleft = new DriveModule(
      DriveConstants.BACKLEFT_MODULE_DRIVE_CAN,
      DriveConstants.BACKLEFT_MODULE_STEER_CAN,
      DriveConstants.BACKLEFT_MODULE_ENCODER,
      DriveConstants.BACKLEFT_MODULE_OFFSET
    );

    //initializing odometry (robot position)
    m_odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), 
      new SwerveModulePosition[] {
        new SwerveModulePosition(frontright.getDriveEncoder(), new Rotation2d()),
        new SwerveModulePosition(frontleft.getDriveEncoder(), new Rotation2d()),
        new SwerveModulePosition(backright.getDriveEncoder(), new Rotation2d()),
        new SwerveModulePosition(backleft.getDriveEncoder(), new Rotation2d())
      }, new Pose2d(0,0, new Rotation2d())
    );
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SmartDashboard.putNumber("P", 0.008);
    SmartDashboard.putNumber("D", 0.0);
    NAVX.reset();
    // navxpid = new PIDController(0.025, 0, 0);
    navxpid = new PIDController(0.005, 0, 0);

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLog log = DataLogManager.getLog();

    //flips the navx, as the robot begins the match facing the driver
    NAVX.setAngleAdjustment(180);

  }
  public void resetFOD() { //reset Field Oriented Drive, ie resets the NAVX
    NAVX.setAngleAdjustment(0);
    NAVX.reset();
  }
  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }
  //resets bot pose to p
  public void resetPose(Pose2d p){
    pose = p;
  }
  //creates a chassisspeeds based off of values (usually from joystick)
  public void drive(double x, double y, double z, int hat) {
    if (autoMode) {return;}

    if (Math.abs(x) < 0.05) {x = 0;}
    if (Math.abs(y) < 0.05) {y = 0;}
    if (Math.abs(z) < 0.05) {z = 0;}
    drive_magnitude = Math.sqrt(x*x+y*y+z*z);

    //chassisSpeeds stores the ideal translation and rotation of the robot.
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, z, NAVX.getRotation2d()); //Use this for field oriented
    // chassisSpeeds = new ChassisSpeeds(y, x, z); //use this for non field oriented

    hatValue = hat; //stores hatvalue for future use
  }
  public void driveAuto(double x, double y, double z) { //drive command for auto with no deadband and no FOD
    chassisSpeeds = new ChassisSpeeds(y, x, z);
  }
  public void driveInDirection(double speed, double angle) { //drives in a specific direction by manually setting the module values
    frontright.set(speed, 0, true);
    frontleft.set(speed, 180, false);
    backright.set(speed, 180, false);
    backleft.set(speed, 0, true);

  }
  //navx is usually off for some reason,sets how many degrees off it is
  public void setPitchOffset(double p) { //Auto platform stuff, pls ignore
    pitchoffset = p;
  }
  
  public double clampToAngle(double i) { //helper method to clamp angle to 0 to 360
    if (i > 360.0) {
      i-=360.0;
    }else if (i < 0.0) {
      i+=360.0;
    }
    return i;
  }
  public boolean shouldSwap(double encoder, double state) { //returns whether to optimize the swerve drive by finding the error between the state and encoder values
    return 180-Math.abs(Math.abs(state-encoder)-180) > 90;
  }

  //swerve optimize switches the direction of the motor when the steer has to rotate more then 90 degrees, based off the currentangle and swerve state
  public SwerveModuleState optimize(
      SwerveModuleState desiredState, double currentAngle) {
    //runs shouldSwap, and uses stateToOptimize to put them both in the same frame of reference
    boolean swap = shouldSwap(stateToOptimize(desiredState.angle.getDegrees()), currentAngle); 
    if (swap) {
      //returns a flipped swerve state
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    }else {
      //returns the original swerve state
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }
  public Translation3d getAccelerometer() { //gets the accelerometer values, pretty self explanatory
    return new Translation3d(accelerometer.getX(), accelerometer.getY(), accelerometer.getZ());
  }
  public Pose2d getPose(){
    return pose;
  }
  public double encoderToOptimize(double encval, float offset) { //puts encoder value into proper frame of reference
    encval -= offset;
    if (encval > 360) {encval-=360;}
    if (encval < 0) {encval+=360;}
    return encval;
  }
  public double stateToOptimize(double stateval) { //flips state value
    // if (stateval > 360) {stateval-=360;}
    // if (stateval < 0) {stateval+=360;}
    stateval += 180;
    return stateval;
  }
  public void setModuleStates(SwerveModuleState[] desiredStates){ //manual set of swerve module states, used for trajectories
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,4.88);
    frontleft.setDesiredState(desiredStates[0],true);
    frontright.setDesiredState(desiredStates[1],false);
    backleft.setDesiredState(desiredStates[2],false);
    backright.setDesiredState(desiredStates[3],true);
  }
  //EXTRMELY BALLER CODE
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath){ //returns command to follow a trajectory (not tested, I hope it works)
    return new SequentialCommandGroup(
      //resets odometry if first trajectory followed
      new InstantCommand(() -> {
        if(isFirstPath){
          this.resetFOD();
        }
      }),
      //swerve trajectory controller
      new PPSwerveControllerCommand(traj, this::getPose , kinematics, new PIDController(DriveConstants.P, 0, 0), new PIDController(DriveConstants.P, 0, 0), new PIDController(DriveConstants.I, 0, 0), this::setModuleStates, this)
    );
  }
  public double getNAVXStraightenVal() { //can use a PID loop to calculate an angle to rotate the robot to avoid drift. Tested but not currently used
    return navxpid.calculate(NAVX.getAngle()-navxoffset, 0);
  }
  public double getDriveEncoderAvg() { //averages all the drive encoders for more convenient auto programming
    return ((frontright.getDriveEncoder() + frontleft.getDriveEncoder() +
            backright.getDriveEncoder() + backleft.getDriveEncoder()/4.0));
  }
  public double getPitch() { //Gets pitch of the robot for autonomous platform
    return NAVX.getPitch()+pitchoffset+10;
  }
  public ChassisSpeeds getChassisSpeeds(){
    return chassisSpeeds;
  }
  public void setChassisSpeeds(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }
  
  @Override
  public void periodic() {
    //takes the kinematics with the module positions and the chassisSpeeds from the drive to give the desired states for each module
    //A swerve module state contains the desired speed and angle for a module
    SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);

    //runs optimize code, explained above. TODO: put offsets into constants
    state[0] = optimize(state[0], encoderToOptimize(backright.getEncoder(), 248));    
    state[1] = optimize(state[1], encoderToOptimize(backleft.getEncoder(), 108));    
    state[2] = optimize(state[2], encoderToOptimize(frontright.getEncoder(), 97));    
    state[3] = optimize(state[3], encoderToOptimize(frontleft.getEncoder(), 64));    

    SmartDashboard.putNumber("PITCH", getPitch());
    if (autoMode == true) {
      return;
    }
    
    //A WHOLE BUNCH OF SMART DASHBOARD DEBUGGING VALUES. Commented out for shuffleboard performance
    
    // SmartDashboard.putNumber("frontright speed: ", Math.abs(state[0].speedMetersPerSecond));
    // // SmartDashboard.putNumber("frontright angle: ", state[0].angle.getDegrees());
    // SmartDashboard.putNumber("frontright encoder: ", frontright.getEncoder());
    // SmartDashboard.putNumber("frontright offset: ", Constants.DriveConstants.FRONTRIGHT_MODULE_OFFSET*360.0);
    // SmartDashboard.putNumber("frontright deviation: ", Math.abs(frontright.drive.getEncoder().getVelocity()));
    // // SmartDashboard.putNumber("frontright current", frontright.drive.getOutputCurrent());

    // SmartDashboard.putNumber("frontleft speed: ", Math.abs(state[1].speedMetersPerSecond));
    // // SmartDashboard.putNumber("frontleft angle: ", state[1].angle.getDegrees());
    // SmartDashboard.putNumber("frontleft encoder: ", frontleft.getEncoder());
    // SmartDashboard.putNumber("frontleft offset: ", Constants.DriveConstants.FRONTLEFT_MODULE_OFFSET*360.0);
    // SmartDashboard.putNumber("frontleft deviation: ", Math.abs(frontleft.drive.getEncoder().getVelocity()));
    // // SmartDashboard.putNumber("frontleft current", frontleft.drive.getOutputCurrent());

    // SmartDashboard.putNumber("backright speed: ", Math.abs(state[2].speedMetersPerSecond));
    // // SmartDashboard.putNumber("backright angle: ", state[2].angle.getDegrees());
    // SmartDashboard.putNumber("backright encoder: ", backright.getEncoder());
    // SmartDashboard.putNumber("backright offset: ", Constants.DriveConstants.BACKRIGHT_MODULE_OFFSET*360.0);
    // SmartDashboard.putNumber("backright deviation: ", Math.abs(backright.drive.getEncoder().getVelocity()));
    // // SmartDashboard.putNumber("backright current", backright.drive.getOutputCurrent());

    // SmartDashboard.putNumber("backleft speed: ", Math.abs(state[3].speedMetersPerSecond));
    // // SmartDashboard.putNumber("backleft angle: ", state[3].angle.getDegrees());
    // SmartDashboard.putNumber("backleft encoder: ", backleft.getEncoder());
    // SmartDashboard.putNumber("backleft offset: ", Constants.DriveConstants.BACKLEFT_MODULE_OFFSET*360.0);
    // SmartDashboard.putNumber("backleft deviation: ", Math.abs(backleft.drive.getEncoder().getVelocity()));
    // // SmartDashboard.putNumber("backleft current", backleft.drive.getOutputCurrent());

    // SmartDashboard.putNumber("ACCELEROMETER X", accelerometer.getX());
    // // SmartDashboard.putNumber("ACCELEROMETER Y", accelerometer.getY());
    // // SmartDashboard.putNumber("ACCELEROMETER Z", accelerometer.getZ());
    SmartDashboard.putNumber("NAVX Angle", NAVX.getAngle());
    SmartDashboard.putNumber("Drive Encoder Avg", getDriveEncoderAvg());
    // if (autoMode) {
    //   return;
    // }
    if (hatValue != -1) { //Allow Creeping the drive with the HAT switch, untested in current branch
      frontright.set(-Constants.DriveConstants.CREEP_SPEED, hatValue, true);
      frontleft.set(Constants.DriveConstants.CREEP_SPEED, hatValue, false);
      backright.set(-Constants.DriveConstants.CREEP_SPEED, hatValue, false);
      backleft.set(-Constants.DriveConstants.CREEP_SPEED, hatValue, true);
    }else {
      //runs the set function for each individual module. Multiplied by max speed, with certain ones flipped.
      frontright.set(state[0].speedMetersPerSecond * Constants.DriveConstants.MAX_SPEED, state[0].angle.getDegrees(), true);
      frontleft.set(state[1].speedMetersPerSecond * Constants.DriveConstants.MAX_SPEED, state[1].angle.getDegrees()+180, false);
      backright.set(state[2].speedMetersPerSecond * Constants.DriveConstants.MAX_SPEED, state[2].angle.getDegrees()+180, false);
      backleft.set(state[3].speedMetersPerSecond * Constants.DriveConstants.MAX_SPEED, state[3].angle.getDegrees(), true);
    }
    
    //gets the swerve drive pose for odometry, measurements untested
    pose = m_odometry.update(new Rotation2d(), 
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontright.getDriveEncoder()*0.1156, new Rotation2d(frontright.getEncoder() * (3.14/180.0))),
      new SwerveModulePosition(frontleft.getDriveEncoder()*0.1156, new Rotation2d(frontleft.getEncoder() * (3.14/180.0))),
      new SwerveModulePosition(backright.getDriveEncoder()*0.1156, new Rotation2d(backright.getEncoder() * (3.14/180.0))),
      new SwerveModulePosition(backleft.getDriveEncoder()*0.1156, new Rotation2d(backleft.getEncoder() * (3.14/180.0)))
    }
    );

    //debug stuff
    SmartDashboard.putNumber("HAT VALUE", hatValue);
    SmartDashboard.putNumber("odometry x", pose.getX());
    SmartDashboard.putNumber("odometry y", pose.getY());
    System.out.println("RUNNING PERIODIC: " + autoMode);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
