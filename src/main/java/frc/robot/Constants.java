// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean debugMode = true;
  public static final boolean pitMode = false;

  public static class CAN {
    //CAN IDs
    public static final int climberElevator = -1;

    public static final int intake = -1;
    public static final int intakePivot = -1;

    public static final int hopper = -1;

    public static final int feeder = -1;

    public static final int shooterLeft = -1;
    public static final int shooterRight = -1;
    
    public static final int shooterHood = -1;

    public static final int Pigeon = 20;
    public static final int PCM = 35;
    public static final int PDP = -1;
  }

  public static class IDs {
    public static final int bottomHooks = -1;
    public static final int topHooks = -1;

    public static final int climberLimit = -1;
    
    public static final int intakeLimit = -1;
    //DIO ports
  }

  public static class Swerve {
    public static double[] translationPID = {5, 0, 0};
    public static double[] rotationPID = {5, 0, 0};
    public static double maxSpeed = 4; // NOT kSpeedAt12Volts
    public static double maxSpeedPartiallyExtended = 1.5;
    public static double maxSpeedFullExtended = 1;
    public static double maxAccelerationFromRest = 7.875; //in m/s^2 gotten from assumed mass of 70 kg
    public static double maxAngularRate = RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 6/10 of a rotation per second max angular velocity
    public static double maxSteerVoltage = 4d;
    public static double maxDriveVoltage = 10d;

    public static double tuningDesiredVelocity = 2d;

    // / 3.54 with 8 volts of voltage compensation and 4.19 with 10 volts
    // 4.8 max speed, 5 acceleration, drops to 9.6
    public final static double kPhysicalMaxSpeedMetersPerSecond = 1;
    public final static double kDeadband = 0.055;
    public final static double kMaxAccelerationDrive = 1.25;
    public final static double kMaxAccelerationAngularDrive = 1.0 * Math.PI;

  }

  // gear ratios and/or ticks per rev, etc.
  public static class SwerveConversions {
    public final static double frontDriveGearRatio = 5.9; // Checked 12/18/24
    public final static double backDriveGearRatio = 6.75; //Checked 12/18/24
    public final static double frontSteerGearRatio = 18.75; // Checked 12/18/24
    public final static double backSteerGearRatio = 21.42857; // Checked 12/18/24
    public static final double wheelDiameter = Units.inchesToMeters(3.9 * 0.96);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public final static double frontDriveRotToMeters = wheelDiameter * Math.PI * (1/(frontDriveGearRatio)); // multiply by
    public final static double backDriveRotToMeters =  wheelDiameter * Math.PI * (1/backDriveGearRatio);
    public static final double frontSteerRotToRads = 1/(frontSteerGearRatio) * Math.PI * 2; // multiply by position
    public static final double backSteerRotToRads = 1/(backSteerGearRatio) * Math.PI * 2; //multiply by position
    public static final double frontDriveRotToMetersPerSecond = frontDriveRotToMeters * 10; // multiply by velocity
    public static final double backDriveRotToMetersPerSecond = backDriveRotToMeters * 10; // multiply by velocity
    public static final double frontSteerRotToRadsPerSecond = frontSteerRotToRads * 10; // multiply by velocity
    public static final double backSteerRotToRadsPerSecond = backSteerRotToRads * 10; // multiply by velocity
  }

  public static class PS5 {
    public static final int BTN_SQUARE = 1;
    public static final int BTN_X = 2;
    public static final int BTN_CIRCLE = 3;
    public static final int BTN_TRIANGLE = 4;
    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;

    public static final int BTN_LJOYSTICK_PRESS = 11;
    public static final int BTN_RJOYSTICK_PRESS = 12;

    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;

    // Gamepad Axis List
    public static final int AXS_LJOYSTICKX = 0;
    public static final int AXS_LJOYSTICKY = 1;
    public static final int AXS_LTRIGGER = 3;
    public static final int AXS_RTRIGGER = 4;
    public static final int AXS_RJOYSTICK_X = 2;
    public static final int AXS_RJOYSTICK_Y = 5;
    }
  
  public static class Xbox {
    // Gamepad Button List
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;

    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;
    public static final int BTN_WINDOW = 7;
    public static final int BTN_MENU = 8;
    public static final int BTN_LJOYSTICK_PRESS = 9;
    public static final int BTN_RJOYSTICK_PRESS = 10;

    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;

    // Gamepad Axis List
    public static final int AXS_LJOYSTICK_X = 0;
    public static final int AXS_LJOYSTICK_Y = 1;
    public static final int AXS_LTRIGGER = 2;
    public static final int AXS_RTRIGGER = 3;
    public static final int AXS_RJOYSTICK_X = 4;
    public static final int AXS_RJOYSTICK_Y = 5;
  }
}
