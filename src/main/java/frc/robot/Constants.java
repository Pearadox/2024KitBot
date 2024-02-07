// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static class DrivetrainConstants {
    public static final int leftFrontID = 10;
    public static final int rightFrontID = 13;
    public static final int leftBackID = 11; 
    public static final int rightBackID = 12;
    
    public static final double deadBan = 0.1;

    public static final int limit = 45;

    public static final double encoderConversionFactor = Units.inchesToMeters(6*Math.PI) / 10.71;
  }
  
  public static class LauncherConstants {
    public static final int feederID = 5;
    public static final int launcherID = 6;

    public static final int launcherCurrentLimit = 80; // change?
    public static final int feederCurrentLimit = 80; // change?

    public static final double launcherSpeed = 1;
    public static final double launchFeederSpeed = 1;
    public static final double intakeLauncherSpeed = -1;
    public static final double intakeFeederSpeed = -0.2;
    public static final double ampLauncherSpeed = 0.17; // 0.14 w/o polycarb bend, from everybot team testing
    public static final double ampFeederSpeed = 0.4; 

    public static final double launcherDelay = .5414*2; // 🤯💀💀💀💀💀💀💀
  }
  
  public static class ClimberConstants {
    public static final int climberID = 9;
    public static final int climberLimit = 45; // change?
    
    public static final double climbUpSpeed = 1;
    public static final double climbDownSpeed = -1;
  }

  public static class RollerClawConstants {
    public static final int rollerClawID = 7;
    public static final int rollerClawLimit = 45; // CIM, brushed

    public static final double clawSpeed = 0.5;
    public static final double clawStallSpeed = 0.1;
    public static final double clawShootSpeed = -0.5;
  }

  public static class AutonConstants {
    
  }
}