// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax mLeftMotor;
  private final CANSparkMax mRightMotor;

  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;

  private double mDesiredSpeed = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mLeftMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    mRightMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);

    mLeftMotor.restoreFactoryDefaults();
    mRightMotor.restoreFactoryDefaults();

    mLeftEncoder = mLeftMotor.getEncoder();
    mRightEncoder = mRightMotor.getEncoder();

    // Sets left to follow the right, allows us to have same speed on both
    mLeftMotor.follow(mRightMotor);

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
