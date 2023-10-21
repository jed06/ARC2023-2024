// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.crypto.dsig.keyinfo.KeyValue;

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

    public static final int LeftDriveTalonPort = 10;
    public static final int RightDriveTalonPort = 10;

    public static final int LeftDriveVictorPort = 4;
    public static final int RightDriveVictorPort = 5;
    public static final int YAxis = 1;
    public static final int XAxis = 0;
    public static final int Zero = 0;
    public static final int One = 1;
    
    
    public static final int leftFlywheelPort = 16;
    public static final int rightFlywheelPort = 17; 
    
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
  public static final class ShooterPorts {
    public static final int LeftFlywheelPort = 16;
    public static final int RightFlywheelPort = 17;
    public static final int pivotPort = 14;
    public static final int rollerPort = 15;
  }

  public static final class USBOrder {
    public static final int Zero = 0;
    public static final int One = 1;
  }

  public static final class leftFlywheelPIDConsts {
    public static double pidP = 0.0;
    public static double pidI = 0.0;
    public static double pidD = 0.0;
    }
    
    public static final class rightFlywheelPIDConsts {
    public static double pidP = 0.0;
    public static double pidI = 0.0;
    public static double pidD = 0.0;
    }
    
    public static final class leftFlywheelFF {
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
      }
      
      public static final class rightFlywheelFF {
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
      }

      public static final class FlywheelSimConstants{
        public static final double kV = 0.37;
        public static final double kA = 0.6;

      } 

  public static final int leftPort = 1;
  public static final int rightPort = 2;

  public static int joy1;
  public static int joy2;
}