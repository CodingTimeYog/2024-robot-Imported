// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LoaderSubsystem;

public class RunLoaderMotor extends Command {
  private final LoaderSubsystem mLoaderSubsystem;
  private final boolean mReverse;

  /** Creates a new runLoaderMotor. */
  public RunLoaderMotor(LoaderSubsystem pLoaderSubsystem, boolean pReverse) {
    mLoaderSubsystem = pLoaderSubsystem;
    mReverse = pReverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLoaderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var loaderSpeed = SmartDashboard.getNumber("Loader Motor Speed", Constants.LOADER_SPEED);
    mLoaderSubsystem.setDesiredLoaderSpeed(loaderSpeed * (mReverse ? -1.0 : 1.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLoaderSubsystem.setDesiredLoaderSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
