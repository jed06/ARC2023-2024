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

  public static final int joyStick1 = 0;
  public static final int joyStick2 = 1;
  public static final int joyStick3 = 2;
  public static final int joyStick4 = 3;

  public static class OperatorConstants {
    

    public static final int kDriverControllerPort = 0;

    public static final int LeftDriveTalonPort = 2;
    public static final int RightDriveTalonPort = 3;

    public static final int LeftDriveVictorPort = 4;
    public static final int RightDriveVictorPort = 5;
    public static final int YAxis = 1;
    public static final int XAxis = 0;
    public static final int Zero = 0;
    public static final int One = 1;
    
    
    public static final int leftFlywheelPort = 30;
    public static final int rightFlywheelPort = 31; 
    
    public static final double EncoderTicksPerRevolution = 4096.0;
  }

  public static class PIDConstants {
    public static final double armPID_P = 0.1;
    public static final double armPID_I = 0.001;
    public static final double armPID_D = 0.0;
    public static final double armPID_K = 2.0; // Used to calculate kP, from the difference in angle
    public static final double extPID_P = 0.05; 
    
  }

  public static class Measurements {
    
    public static double maxDriveSpeed = 0.6;

  }

  public static class ButtonMap {

// Buttons can be assigned to functions here  
  }


  public static final int leftPort = 1;
  public static final int rightPort = 2;

  public static int joy1;
  public static int joy2;
}