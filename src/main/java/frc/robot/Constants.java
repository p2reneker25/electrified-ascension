// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class DriveConstants {
        public static final int FRONTRIGHT_MODULE_DRIVE_CAN = 3;
        public static final int FRONTRIGHT_MODULE_STEER_CAN = 2; 
        public static final int FRONTRIGHT_MODULE_ENCODER = 2; 
        public static final double FRONTRIGHT_MODULE_OFFSET = 0.77; 

        public static final int FRONTLEFT_MODULE_DRIVE_CAN = 4;
        public static final int FRONTLEFT_MODULE_STEER_CAN = 9; 
        public static final int FRONTLEFT_MODULE_ENCODER = 0; 
        public static final double FRONTLEFT_MODULE_OFFSET = 0.18; 

        public static final int BACKRIGHT_MODULE_DRIVE_CAN = 8;
        public static final int BACKRIGHT_MODULE_STEER_CAN = 5; 
        public static final int BACKRIGHT_MODULE_ENCODER = 3; 
        public static final double BACKRIGHT_MODULE_OFFSET = 0.69; 

        public static final int BACKLEFT_MODULE_DRIVE_CAN = 10;
        public static final int BACKLEFT_MODULE_STEER_CAN = 1; 
        public static final int BACKLEFT_MODULE_ENCODER = 1; 
        public static final double BACKLEFT_MODULE_OFFSET = 0.8; 


        public static final double MAX_SPEED = 0.1;
        public static final double CHASSIS_WIDTH = 0.508;
        public static final double CHASSIS_LENGTH = 0.6223;
        
        public static final double P = 0.012;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }
    public final class ClawConstants {
        public static final int ENCODER_TICKS = 42;
        public static final int WRIST_CAN = 5;
        public static final double CONST_SPEED = 0.3;
    }
    public final class ButtonConstants {
        public static final int BUTTON_WRIST = 2;
        public static final int BUTTON_ARMFORWARD = 3;
        public static final int BUTTON_ARMBACKWARD = 4;
        public static final int BUTTON_ARMUP = 5;
        public static final int BUTTON_ARMDOWN = 6;

        public static final int BUTTON_ARM_EXTEND = 1; 
        public static final int BUTTON_ARM_RETRACT = 2; 
        public static final int BUTTON_POSITION = 2;
    }
   
    public final class ArmConstants {
        public static final int EXTEND_MOTOR_CAN = 11;
        public static final int BRAKE_1 = 1;
        public static final int BRAKE_2 = 2;
        public static final int ARM_PWM = 0;
        public static final int PIVOT_CAN = 12;

        public static final double ARM_SPEED = 0.15;
    }
    public final class HandConstants {
        public static final int HAND_MOTOR_PWM = 1;
    }
}  
