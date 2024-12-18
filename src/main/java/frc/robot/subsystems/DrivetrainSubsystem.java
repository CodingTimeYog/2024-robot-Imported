// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   *
   * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
   * initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = Constants.MAX_Voltage;

  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiamefszter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */

  // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
  // SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      Constants.FALCON_500_FREE_SPINNING_RPM
          / 60.0
          * SdsModuleConfigurations.MK4_L2.getDriveReduction()
          * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
          * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics mKinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final Pigeon2 mPigeon = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule mFrontLeftModule;
  private final SwerveModule mFrontRightModule;
  private final SwerveModule mBackLeftModule;
  private final SwerveModule mBackRightModule;
  // private final SwerveDriveOdometry m_odometry;
  private Pose2d mPose = new Pose2d();
  private Field2d mField = new Field2d();

  private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] mStates = mKinematics.toSwerveModuleStates(mChassisSpeeds);

  private final SwerveDrivePoseEstimator mPoseEstimator;

  private final ShuffleboardLayout mFrontLeftModuleLayout;
  private final ShuffleboardLayout mFrontRightModuleLayout;
  private final ShuffleboardLayout mBackLeftModuleLayout;
  private final ShuffleboardLayout mBackRightModuleLayout;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add("Field", mField);

    var moduleConfig = new Mk4ModuleConfiguration();
    moduleConfig.setDriveCurrentLimit(60.0);

    mFrontLeftModuleLayout =
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    mFrontLeftModule =
        Mk4iSwerveModuleHelper.createFalcon500Neo(
            // This parameter is optional, but will allow you to see the current state of
            // the module on the dashboard.
            mFrontLeftModuleLayout,
            moduleConfig,
            // Prabhu- Use Mk4i - L2 as this is what 2186 Team purchased
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    mFrontRightModuleLayout =
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    mFrontRightModule =
        Mk4iSwerveModuleHelper.createFalcon500Neo(
            mFrontRightModuleLayout,
            moduleConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    mBackLeftModuleLayout =
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
    mBackLeftModule =
        Mk4iSwerveModuleHelper.createFalcon500Neo(
            mBackLeftModuleLayout,
            moduleConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    mBackRightModuleLayout =
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);
    mBackRightModule =
        Mk4iSwerveModuleHelper.createFalcon500Neo(
            mBackRightModuleLayout,
            moduleConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            mKinematics,
            getGyroscopeRotation(),
            getSwerveModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    AutoBuilder.configureHolonomic(
        this::getCurrentPose,
        this::resetOdometry,
        this::getChassisSpeedsRobotRelative,
        this::setChassisSpeedsRobotRelative,
        Constants.PATH_FOLLOWER_CONFIG,
        this::shouldFlip,
        this);

    addTemperatureToLayout(mFrontLeftModuleLayout, mFrontLeftModule);
    addTemperatureToLayout(mFrontRightModuleLayout, mFrontRightModule);
    addTemperatureToLayout(mBackLeftModuleLayout, mBackLeftModule);
    addTemperatureToLayout(mBackRightModuleLayout, mBackRightModule);

    tab.addNumber("Pigeon Yaw", mPigeon::getYaw);
  }

  /**
   * @return The current estimated pose of the drivetrain.
   */
  public Pose2d getCurrentPose() {
    return mPose;
  }

  /**
   * @return The current heading of the drivetrain, and subsequently robot.
   */
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(mPigeon.getYaw());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    mPigeon.setYaw(0.0);
  }

  public void resetOdometry(Pose2d pNewDefaultPose) {
    mPoseEstimator.resetPosition(
        getGyroscopeRotation(), getSwerveModulePositions(), pNewDefaultPose);
  }

  public ChassisSpeeds getChassisSpeedsRobotRelative() {
    return mKinematics.toChassisSpeeds(getCurrentModuleStates());
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getChassisSpeedsRobotRelative(), getGyroscopeRotation());
  }

  public void setChassisSpeeds(ChassisSpeeds pSpeeds) {
    mChassisSpeeds = pSpeeds;
  }

  public void setChassisSpeedsRobotRelative(ChassisSpeeds pChassisSpeeds) {
    mChassisSpeeds = pChassisSpeeds;
  }

  public void setChassisSpeedsFieldRelative(ChassisSpeeds pChassisSpeeds) {
    mChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(pChassisSpeeds, getGyroscopeRotation());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      makePosition(mFrontLeftModule),
      makePosition(mFrontRightModule),
      makePosition(mBackLeftModule),
      makePosition(mBackRightModule)
    };
  }

  public SwerveModuleState[] getCurrentModuleStates() {
    return mStates;
  }

  @Override
  public void periodic() {
    updateOdometry();

    mFrontLeftModule.set(
        mStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        mStates[0].angle.getRadians());
    mFrontRightModule.set(
        mStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        mStates[1].angle.getRadians());
    mBackLeftModule.set(
        mStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        mStates[2].angle.getRadians());
    mBackRightModule.set(
        mStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        mStates[3].angle.getRadians());
  }

  private void updateOdometry() {
    mStates = mKinematics.toSwerveModuleStates(mChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(mStates, MAX_VELOCITY_METERS_PER_SECOND);

    mPoseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
    var visionMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (visionMeasurement.tagCount >= 2) {
      mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      mPoseEstimator.addVisionMeasurement(
          visionMeasurement.pose, visionMeasurement.timestampSeconds);
    }

    mPose = mPoseEstimator.getEstimatedPosition();
    mField.setRobotPose(mPose);
  }

  private SwerveModulePosition makePosition(SwerveModule pModule) {
    return new SwerveModulePosition(
        pModule.getDriveDistance(), new Rotation2d(pModule.getSteerAngle()));
  }

  private boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.map(a -> a.equals(Alliance.Red)).orElse(false);
  }

  private static DoubleSupplier makeDriveMotorTemperatureSupplier(SwerveModule pModule) {
    // noinspection removal
    var driveMotor = (WPI_TalonFX) pModule.getDriveMotor();
    return driveMotor::getTemperature;
  }

  private static DoubleSupplier makeSteerMotorTemperatureSupplier(SwerveModule pModule) {
    var steerMotor = (CANSparkMax) pModule.getSteerMotor();
    return steerMotor::getMotorTemperature;
  }

  private static void addTemperatureToLayout(ShuffleboardLayout pLayout, SwerveModule pModule) {
    pLayout.addNumber("Drive Motor Temperature", makeDriveMotorTemperatureSupplier(pModule));
    pLayout.addNumber("Steer Motor Temperature", makeSteerMotorTemperatureSupplier(pModule));
  }
}
