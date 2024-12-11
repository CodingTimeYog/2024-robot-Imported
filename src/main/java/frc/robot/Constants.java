// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.64;

  public static final double FALCON_500_FREE_SPINNING_RPM = 6380;
  public static final double NEO_FREE_SPINNING_RPM = 5880;

  public static final int SPARK_SHAFT_ENCODER_COUNT_PER_ROTATION = 8192;

  /** The RPM of the drive motors on a swerve drive module when there is no load. */
  /** The RPM of the drive motors on a swerve drive module when there is no load. */
  public static final double SWERVE_DRIVE_MOTOR_FREE_RPM = FALCON_500_FREE_SPINNING_RPM;

  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.63; // FIXME Measure and set wheelbase

  public static final double kP = 0.03;
  public static final double kI = 0.0;
  public static final double kD = 0.05;

  public static final int DRIVETRAIN_PIGEON_ID = 19;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 17;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(23.642578);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 18;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(316.93359);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(25.04882);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 15;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(5.800781);

//   public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
//       new HolonomicPathFollowerConfig(
//           new PIDConstants(5.1275, 0.755),
//           new PIDConstants(1.985),
//           5.0,
//           DRIVETRAIN_WHEELBASE_METERS,
//           new ReplanningConfig());

  //
  // Shooter Motor Constants
  public static final int SHOOTER_MOTOR_LEFT = 10;
  public static final int SHOOTER_MOTOR_RIGHT = 2;

  public static final double SHOOTER_MIN_VELOCITY_RPM = 2400.0;

  public static final double SHOOTER_MAX_VELOCITY = 0;

  // Loader Motor Constants
  public static final int LOADER_MOTOR_CAN_ID = 21;

  public static final double LOADER_SPEED = 0.3;

  // Intake Motor Constants
  public static final int INTAKE_MOTOR_CAN_ID = 22;

  public static final double INTAKE_SPEED = 0.35;

  // Arm Motor Constants
  public static final int ARM_LIMIT_SWITCH_LOWER_DIO_PORT = 0;
  public static final int ARM_LIMIT_SWITCH_UPPER_DIO_PORT = 1;

  public static final double ARM_SPEED = 0.5;

  public static final int ARM_LEFT_MOTOR_ID = 1;
  public static final int ARM_RIGHT_MOTOR_ID = 30;

  public static final double ARM_MAX_SAFE_ANGLE_DEGREES = 20;

  // LED Strip Constant
  public static final int LED_STRIP_CAN_ID = 0;
  public static final int LED_STRIP_NUM_LEDS = 46;

  // Prabhu - Max voltage changed from 12 to 2
  public static double MAX_Voltage = 4;

  public static final int DRIVER_CONTROLLER_ID = 0;

  public static final int OPERATOR_CONTROLLER_ID = 1;

  public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

  public static final int LIFTER_SOLENOID_A_VALVE_ID = 1;
  public static final int LIFTER_SOLENOID_B_VALVE_ID = 0;

  public static final int COMPRESSOR_CAN_ID = 15;

  public static final int BEAM_BREAKER_DIO_PORT = 3;

  public static final Angle ARM_LOWER_POSITION_ABSOLUTE_ENCODER_OFFSET =
      Units.Degrees.of(248.5);

public static final double LEFT_Y_DEADBAND = 0.05;

public static final double LEFT_X_DEADBAND = 0.05;
}
