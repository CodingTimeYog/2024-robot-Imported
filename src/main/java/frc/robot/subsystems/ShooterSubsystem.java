// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax mLeftMotor;
  private final SparkMax mRightMotor;

  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;

  private double mDesiredSpeed = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mLeftMotor = new SparkMax(Constants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    mRightMotor = new SparkMax(Constants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);
    
    // Setting Config for things like motor following, current limit,
    // and motor mode. 
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.apply(globalConfig).follow(mRightMotor);
    mRightMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    mLeftEncoder = mLeftMotor.getEncoder();
    mRightEncoder = mRightMotor.getEncoder();


    Shuffleboard.getTab("Arm").addNumber("Flywheel Speed (RPM)", mLeftEncoder::getVelocity);
  }

  public void stopMotors() {
    mDesiredSpeed = 0.0;
  }

  public void setMotor(double pSpeed) {
    mDesiredSpeed = pSpeed;
  }

  public double getEncoderRotation() {
    return mRightEncoder.getPosition();
  }

  // FIXME Need to change constant for minimum velocity
  public boolean isShooterUpToSpeed() {
    return Math.abs(mRightEncoder.getVelocity()) >= Constants.SHOOTER_MIN_VELOCITY_RPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mRightMotor.set(mDesiredSpeed * -1);
  }
}
