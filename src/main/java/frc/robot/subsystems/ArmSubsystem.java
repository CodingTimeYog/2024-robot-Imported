package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Variables

  private final SparkMax mLeftMotor;
  private final SparkMax mRightMotor;

  private final ArmFeedforward mArmFeedforward = new ArmFeedforward(0.74621, 0, 1.25);

  private SparkAbsoluteEncoder mEncoder;

  private final DigitalInput mLimitSwitchBottom =
      new DigitalInput(Constants.ARM_LIMIT_SWITCH_LOWER_DIO_PORT);
  private final DigitalInput mLimitSwitchTop =
      new DigitalInput(Constants.ARM_LIMIT_SWITCH_UPPER_DIO_PORT);

  private final SysIdRoutine mSysIdRoutine;
  private final MedianFilter mVelocityFilter = new MedianFilter(32);

  private final ProfiledPIDController mPidController;

  private double mMotorValue = 0.0;
  private boolean mPidControllerEnabled = false;
  SparkMaxConfig config = new SparkMaxConfig();

  /// Arm component of the robot
  public ArmSubsystem() {
    mLeftMotor = new SparkMax(Constants.ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
    mRightMotor = new SparkMax(Constants.ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);
    // mRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // mRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    mEncoder = mRightMotor.getAbsoluteEncoder();
    config.signals.primaryEncoderPositionPeriodMs(5);
    config.absoluteEncoder.averageDepth(64);
    config.absoluteEncoder.zeroOffset(Constants.ARM_LOWER_POSITION_ABSOLUTE_ENCODER_OFFSET.in(Units.Rotation));
    config.idleMode(IdleMode.kBrake);
    // mLeftMotor.setIdleMode(IdleMode.kBrake);
    // mRightMotor.setIdleMode(IdleMode.kBrake);
    var stepRate = Units.Volts.of(1);
    var stepVoltage = Units.Volts.of(4);
    var timeout = Units.Seconds.of(60);
    
    mSysIdRoutine =
        new SysIdRoutine(
            new Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  var voltage = volts.in(Units.Volts);

                  SmartDashboard.putNumber("Arm: Raw Voltage", voltage);

                  mRightMotor.setVoltage(-voltage);
                  mLeftMotor.setVoltage(voltage);
                },
                this::handleSysIdLog,
                this));

    mPidController =
        new ProfiledPIDController(0, 0.0, 0, new TrapezoidProfile.Constraints(20.0, 2.5));
    mPidController.setGoal(getEncoderPositionDegrees());
    mPidController.setTolerance(0.25);

    var tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Setpoint", this::getValue);
    tab.addNumber(
        "Right Applied Voltage",
        () -> mRightMotor.getAppliedOutput() * mRightMotor.getBusVoltage());
    tab.addNumber(
        "Left Applied Voltage", () -> mLeftMotor.getAppliedOutput() * mLeftMotor.getBusVoltage());
    tab.addNumber("Encoder Position (Rotations)", () -> getArmPosition().in(Units.Rotations));
    tab.addNumber("Encoder Position (Degrees)", () -> getEncoderPositionDegrees());
    tab.addNumber(
        "Encoder Velocity (Rotations)", () -> getArmVelocity().in(Units.RotationsPerSecond));
    tab.addNumber("Encoder Velocity (Degrees)", () -> getArmVelocity().in(Units.DegreesPerSecond));
    tab.addBoolean("Is the Arm in Safe position to fire", () -> isArmSafe());
    tab.addBoolean("At Bottom", this::atBottom);
    tab.addBoolean("At Top", this::atTop);
    tab.add("Top Limit Switch", mLimitSwitchTop);
    tab.add("PID Controller", mPidController);

    mLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getValue() {
    return mMotorValue;
  }

  public void setValue(double pMotorSpeed) {
    mMotorValue = pMotorSpeed;
  }

  public void enableBangBang() {
    // mLeftMotor.setIdleMode(IdleMode.kCoast);
    // mRightMotor.setIdleMode(IdleMode.kCoast);
    config.idleMode(IdleMode.kCoast);
    mLeftMotor.configure(config, null, null);
    mRightMotor.configure(config, null, null);
  }

  public void disableBangBang() {
    config.idleMode(IdleMode.kCoast);
    mLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightMotor.configure(config, null, null);

    // mLeftMotor.setIdleMode(IdleMode.kBrake);
    // mRightMotor.setIdleMode(IdleMode.kBrake);
  }

  // encoder funcs
  public Angle getArmPosition() {
    return Units.Rotations.of(mEncoder.getPosition());
  }

  public double getEncoderPositionDegrees() {
    return getArmPosition().in(Units.Degrees);
  }

  public AngularVelocity getArmVelocity() {
    var velocityRotationsPerSecond = mEncoder.getVelocity();
    var filteredVelocity = mVelocityFilter.calculate(velocityRotationsPerSecond);
    return Units.RotationsPerSecond.of(filteredVelocity);
  }

  public boolean isArmSafe() {
    return getEncoderPositionDegrees() <= Constants.ARM_MAX_SAFE_ANGLE_DEGREES;
  }

  public boolean atBottom() {
    return !mLimitSwitchBottom.get();
  }

  public boolean atTop() {
    return mLimitSwitchTop.get();
  }

  public Angle getDesiredAngle() {
    return Units.Degrees.of(mPidController.getGoal().position);
  }

  public void setDesiredAngle(Angle pAngle) {
    mPidController.setGoal(pAngle.in(Units.Degrees));
  }

  public boolean atDesiredAngle() {
    return mPidController.atGoal();
  }

  public void usePidController() {
    mPidControllerEnabled = true;
    mPidController.reset(getEncoderPositionDegrees());
  }

  public void disablePidController() {
    mPidControllerEnabled = false;
    mMotorValue = 0.0;
  }

  public void resetRateLimit() {}

  @Override
  public void periodic() {
    if (mPidControllerEnabled) {
      var setpoint = mPidController.getSetpoint();
      var pidOutput = mPidController.calculate(getEncoderPositionDegrees());
      var feedforward =
          mArmFeedforward.calculate(
              getArmPosition().in(Units.Radians), getArmVelocity().in(Units.RadiansPerSecond));

      SmartDashboard.putNumber("Arm: Raw Setpoint - Position", setpoint.position);
      SmartDashboard.putNumber("Arm: Raw Setpoint - Velocity", setpoint.velocity);
      SmartDashboard.putNumber("Arm: Raw PID Output", pidOutput);
      SmartDashboard.putNumber("Arm: Raw Feedforward", feedforward);

      var motorOutput = pidOutput;
      setMotorValues(motorOutput);
    } else {
      setMotorValues(mMotorValue * 12.0);
    }
  }

  public void stopMotor() {
    mRightMotor.set(0);
  }

  public Command sysidQuasistatic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mSysIdRoutine.quasistatic(pDirection).until(() -> atTop());
    } else {
      return mSysIdRoutine.quasistatic(pDirection).until(() -> atBottom());
    }
  }

  public Command sysidDynamic(SysIdRoutine.Direction pDirection) {
    if (pDirection == SysIdRoutine.Direction.kForward) {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atTop());
    } else {
      return mSysIdRoutine.dynamic(pDirection).until(() -> atBottom());
    }
  }

  private void handleSysIdLog(SysIdRoutineLog pLog) {
    var rightMotorVolts =
        Units.Volts.of(mRightMotor.getAppliedOutput() * mRightMotor.getBusVoltage());

    pLog.motor("arm")
        .voltage(rightMotorVolts.negate())
        .angularPosition(getArmPosition())
        .angularVelocity(getArmVelocity());
  }

  private void setMotorValues(double pMotorValue) {
    if (pMotorValue > 0.0 && atTop()) {
      pMotorValue = 0.0;
    }
    if (pMotorValue < 0.0 && atBottom()) {
      pMotorValue = 0.0;
    }

    SmartDashboard.putNumber("Arm: Raw Voltage", pMotorValue);

    mRightMotor.setVoltage(-pMotorValue);
    mLeftMotor.setVoltage(pMotorValue);
  }
}
