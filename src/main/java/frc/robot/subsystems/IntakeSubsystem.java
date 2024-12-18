// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private RelativeEncoder mIntakeEncoder;

  private final SparkMax mIntakeMotor;

  private double mDesiredIntakeSpeed = 0.0;

  // SysID:
  private final MutVoltage mAppliedIntakeVoltage = (MutVoltage) (Volts.of(0));
  private final MutVoltage mAppliedLoaderVoltage = (MutVoltage) Volts.of(0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    mIntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig mIntakeConfig = new SparkMaxConfig();
    mIntakeConfig.inverted(true);

    mIntakeEncoder = mIntakeMotor.getEncoder();
  }

  public double getIntakeSpeedSetpoint() {
    return mDesiredIntakeSpeed;
  }

  public void setIntakeSpeedSetpoint(double pIntakeSpeed) {
    mDesiredIntakeSpeed = pIntakeSpeed;
  }

  // // To command
  // public void runIntakeMotors(double speed) {
  //   mIntakeMotor.set(speed);
  // }

  public void stopIntakeMotors() {
    mDesiredIntakeSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mIntakeMotor.set(mDesiredIntakeSpeed);
  }
}
