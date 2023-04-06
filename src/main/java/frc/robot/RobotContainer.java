// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoBalence;
import frc.robot.commands.AutoSequence;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HandGrab;
import frc.robot.commands.Intake;
import frc.robot.commands.ResetFOD;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Jackson9001;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain drivetrain = new DriveTrain();
  //private final Wrist wrist = new Wrist();
  private final Vision vision = new Vision();
  // private final Arm arm = new Arm();
  private final Hand hand = new Hand();
  private final Jackson9001 roller = new Jackson9001();
  private final Joystick joystick;
  private final Joystick joystick2;
  private final Joystick guitar;
  private final JoystickButton hand_grab;
  private final JoystickButton hand_release;
  private final JoystickButton roller_intake;
  private final JoystickButton roller_outtake;
  private final JoystickButton drive_forward;
  private final JoystickButton drive_resetfod;


  // private final JoystickButton arm_setGround;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

      // Configure the button bindings
    joystick = new Joystick(0);
    joystick2 = new Joystick(1);
    guitar = new Joystick(2);
    // b_armExtend = new JoystickButton(joystick, ButtonConstants.BUTTON_ARM_EXTEND);
    // b_armRetract = new JoystickButton(joystick, ButtonConstants.BUTTON_ARM_RETRACT);
    // b_positionrobot = new JoystickButton(joystick, ButtonConstants.BUTTON_POSITION);
    //b_turnWrist = new JoystickButton(joystick, Constants.ButtonConstants.BUTTON_WRIST);
    hand_grab = new JoystickButton(guitar, ButtonConstants.BUTTON_HAND_HOLD_GUITAR);
    hand_release = new JoystickButton(guitar, ButtonConstants.BUTTON_HAND_LETGO_GUITAR);
    roller_intake = new JoystickButton(guitar, ButtonConstants.BUTTON_INTAKE_GUITAR);
    roller_outtake = new JoystickButton(guitar, ButtonConstants.BUTTON_OUTTAKE_GUITAR);
    drive_resetfod = new JoystickButton(joystick, ButtonConstants.BUTTON_RESETFOD);
    // hand_close = new JoystickButton(guitar,ButtonConstants.BUTTON_HAND_OPEN_GUITAR);
    // hand_open = new JoystickButton(guitar, ButtonConstants.BUTTON_HAND_OPEN_GUITAR);
    // arm_backwards = new JoystickButton(guitar, Constants.ButtonConstants.BUTTON_ARM_RETRACT_GUITAR);
    // arm_forwards = new JoystickButton(guitar, Constants.ButtonConstants.BUTTON_ARM_EXTEND_GUITAR);
    // arm_up = new JoystickButton(guitar, Constants.ButtonConstants.BUTTON_ARMUP_GUITAR);
    // arm_down = new JoystickButton(guitar, Constants.ButtonConstants.BUTTON_ARMDOWN_GUITAR);
    // b_break  = new JoystickButton(joystick, Constants.ButtonConstants.BUTTON_BRAKE);
    drive_forward = new JoystickButton(joystick, Constants.ButtonConstants.BUTTON_DRIVE_FORWARD);

    // arm_setGround = new JoystickButton(joystick, Constants.ButtonConstants.BUTTON_SETGROUND);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //b_turnWrist.onTrue(new TurnWrist(wrist));
    // arm_backwards.whileTrue(new ArmBackward(arm));
    // arm_forwards.whileTrue(new ArmForward(arm));
    hand_grab.whileTrue(new HandGrab(hand,-0.5));
    hand_release.whileTrue(new HandGrab(hand,1.0));
    roller_intake.whileTrue(new Intake(roller,0.25));
    roller_outtake.whileTrue(new Intake(roller, -0.25));
    // hand_close.onTrue(new HandCylinder(hand));
    // hand_open.onTrue(new HandCylinder(hand));
    // arm_up.whileTrue(new PivotArm(arm, Constants.ArmConstants.ARM_SPEED));
    // arm_down.whileTrue(new PivotArm(arm,-Constants.ArmConstants.ARM_SPEED * 0.5));
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, joystick, joystick2));
    drive_forward.whileTrue(new AutoBalence(drivetrain));
    drive_resetfod.onTrue(new ResetFOD(drivetrain));
    // b_positionrobot.whileTrue(new PositionRobot(drivetrain, vision));
    // b_break.whileTrue(new Brake(arm));
    // arm_setGround.whileTrue(new ArmSetGround(arm));
    // b_armExtend.whileTrue(new ExtendArm(arm,0.1));
    // b_armRetract.whileTrue(new ExtendArm(arm,-0.1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
    arm_up.whileTrue(new PivotArm(arm, Constants.ArmConstants.ARM_SPEED));
    arm_down.whileTrue(new PivotArm(arm,-Constants.ArmConstants.ARM_SPEED * 0.5));
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, joystick));
    b_positionrobot.whileTrue(new PositionRobot(drivetrain, vision));
    // b_armExtend.whileTrue(new ExtendArm(arm,0.1));
    // b_armRetract.whileTrue(new ExtendArm(arm,-0.1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutoSequence(drivetrain, hand);
  }
}
