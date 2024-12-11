// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LoaderSubsystem extends SubsystemBase {
  private final CANSparkMax mLoaderMotor;
  private final RelativeEncoder mEncoder;
  private final DigitalInput mBeam;
  private double mDesiredLoaderSpeedSetpoint = 0.0;

  /** Creates a new LoaderSubsystem. */
  public LoaderSubsystem() {
    mLoaderMotor = new CANSparkMax(Constants.LOADER_MOTOR_CAN_ID, MotorType.kBrushless);
    mLoaderMotor.restoreFactoryDefaults();
    mLoaderMotor.setInverted(true);
    mLoaderMotor.setIdleMode(IdleMode.kBrake);
    mEncoder = mLoaderMotor.getEncoder();
    mBeam = new DigitalInput(Constants.BEAM_BREAKER_DIO_PORT);

    var tab = Shuffleboard.getTab("Loader");
    tab.addBoolean("Has Note", () -> !mBeam.get());
    tab.addDouble("Motor Setpoint", () -> mDesiredLoaderSpeedSetpoint);
    tab.addDouble("Motor RPM", mEncoder::getVelocity);
  }

  public double getDesiredLoaderSpeed() {
    return mDesiredLoaderSpeedSetpoint;
  }

  public void setDesiredLoaderSpeed(double speed) {
    mDesiredLoaderSpeedSetpoint = speed;
  }

  public void stopLoaderMotors() {
    mDesiredLoaderSpeedSetpoint = 0.0;
  }

  public boolean hasNote() {
    return !mBeam.get();
  }

  public AngularVelocity getLoaderSpeed() {
    return Units.RPM.of(mEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    mLoaderMotor.set(mDesiredLoaderSpeedSetpoint);
  }
}
