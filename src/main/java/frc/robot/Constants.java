// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private static final double robotWeight = 138; //
  public static final double ROBOT_MASS = robotWeight * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag // Old 0.13; 0.02 for saftey.
                                               // TEST 0.012
  public static final double MAX_SPEED = 4.0;

  public static final double L1RELEASESPEED = -0.25;
  public static final double RELEASESPEED = -0.5;
  public static final double INTAKESPEED = -0.13;

  public static final double CLIMBERSPEED = 0.5;

  public static final double X_REEF_ALIGNMENT_P = 2.5;
  public static final double Y_REEF_ALIGNMENT_P = 3.0;
  public static final double ROT_REEF_ALIGNMENT_P = 0.058;

  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -0.375; // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.01;
  public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = -0.45; // Horizontal pose
  public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = -0.11;

  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.005;

  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double ALIGN_TIMEOUT = 1.5;
  public static final double POSE_VALIDATION_TIME = 0.3;

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ConfingValues {
    public static final int ClimberCANID = 9;
    public static final int IntakeCANID = 12;
    public static final int ELEVATORLEFTCANID = 11;
    public static final int ELEVATORRIGHTCANID = 10;
    public static final int InnerLaserCANID = 2;
    public static final int OuterLaserCANCANID = 1;
    public static final int LightsPWMPORT = 9;
    public static final int LIGHTSBUFFERSIZE = 12;

    public static final double ELEVATOR_MIN_HEIGHT = 0.5;
    public static final double ELEVATOR_MAX_HEIGHT = 27;

  }

  public static final double L1_HEIGHT = 12;
  public static final double L2_HEIGHT = 6.25;
  public static final double L3_HEIGHT = 13.75;
  public static final double L4_HEIGHT = 27;

  public static enum Levels {
    Home,
    L1,
    L2,
    L3,
    L4
  }

}