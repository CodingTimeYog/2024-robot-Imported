// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.*;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain_Subsystem_Second extends SubsystemBase {
  /** Creates a new Drivetrain_Subsystem_Second. */

  private final SwerveDrive mSwerveDrive;

  public double maxSpeed = Units.feetToMeters(14.5);
  private final ShuffleboardLayout mFrontLeftModuleLayout;
  private final ShuffleboardLayout mFrontRightModuleLayout;
  private final ShuffleboardLayout mBackLeftModuleLayout;
  private final ShuffleboardLayout mBackRightModuleLayout;

    /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  private Field2d mField = new Field2d();

  public Drivetrain_Subsystem_Second(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.428571);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    System.out.println("Before try catch");
    try
    {
      System.out.println("In try catch");
      // mSwerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
       mSwerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    System.out.println("Out of try catch");
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add("Field", mField);
    mFrontLeftModuleLayout =
      tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    mFrontRightModuleLayout =
      tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    mBackLeftModuleLayout =
      tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
    mBackRightModuleLayout =
      tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);

    addVoltageToLayout(mFrontLeftModuleLayout, mSwerveDrive.getModules()[0]);
    addVoltageToLayout(mFrontRightModuleLayout, mSwerveDrive.getModules()[1]);
    addVoltageToLayout(mBackLeftModuleLayout, mSwerveDrive.getModules()[2]);
    addVoltageToLayout(mBackRightModuleLayout, mSwerveDrive.getModules()[3]);

    mSwerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    mSwerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    setupPathPlanner(); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
  }



  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        Constants.PATH_FOLLOWER_CONFIG
        ,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
  }
  public Pose2d getPose()
  {
    return mSwerveDrive.getPose();
  }
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    mSwerveDrive.resetOdometry(initialHolonomicPose);
  }
  public ChassisSpeeds getFieldVelocity()
  {
    return mSwerveDrive.getFieldVelocity();
  }
  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return mSwerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    mSwerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro()
  {
    mSwerveDrive.zeroGyro();
    mSwerveDrive.resetOdometry(getPose());
  }
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public Rotation2d getPitch()
  {
    return mSwerveDrive.getPitch();
  }
  
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      mSwerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * mSwerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * mSwerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * mSwerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  private boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.map(a -> a.equals(Alliance.Red)).orElse(false);
  }
  private static DoubleSupplier makeDriveMotorVolatageSupplier(swervelib.SwerveModule pModule) {
    // Use a lambda to return the double as a DoubleSupplier
    var driveMotor = pModule.getDriveMotor();
    return () -> driveMotor.getVoltage(); // Wrap in lambda to match DoubleSupplier type
}


private static DoubleSupplier makeAngleMotorTemperatureSupplier(swervelib.SwerveModule pModule) {
  // Use a lambda to return the double as a DoubleSupplier
  var driveMotor = pModule.getAngleMotor();
  return () -> driveMotor.getVoltage(); // Wrap in lambda to match DoubleSupplier type
}

  private static void addVoltageToLayout(ShuffleboardLayout pLayout, swervelib.SwerveModule swerveModule) {
    pLayout.addNumber("Drive Motor Temperature", makeDriveMotorVolatageSupplier(swerveModule));
    pLayout.addNumber("Steer Motor Temperature", makeAngleMotorTemperatureSupplier(swerveModule));
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    mSwerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }
  public void drive(ChassisSpeeds velocity)
  {
    mSwerveDrive.drive(velocity);
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(mSwerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      mSwerveDrive.getOdometryHeading().getRadians(),
                                                                      mSwerveDrive.getMaximumVelocity()));
    });
  }
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    mSwerveDrive.driveFieldOriented(velocity);
  }

  public SwerveDriveKinematics getKinematics()
  {
    return mSwerveDrive.kinematics;
  }

  public void postTrajectory(Trajectory trajectory)
  {
    mSwerveDrive.postTrajectory(trajectory);
  }
  public void setMotorBrake(boolean brake)
  {
    mSwerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void periodic() {
  }
}
