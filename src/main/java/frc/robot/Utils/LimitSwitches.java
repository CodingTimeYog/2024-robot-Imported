// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimitSwitches extends SubsystemBase {
  private DigitalInput lowerLim;

  private DigitalInput upperLim;

  public LimitSwitches() {
    lowerLim = new DigitalInput(Constants.ARM_LIMIT_SWITCH_LOWER_DIO_PORT);
    upperLim = new DigitalInput(Constants.ARM_LIMIT_SWITCH_UPPER_DIO_PORT);
  }

  public int getChannelUpper() {
    return upperLim.getChannel();
  }

  public int getChannelLower() {
    return lowerLim.getChannel();
  }

  public boolean isUpperLimitHit() {
    return upperLim.get();
  }

  public boolean isLowerLimitHit() {
    return lowerLim.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
