// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeMotors extends Command {
  /** Creates a new RunIntakeMotors. */
  private final IntakeSubsystem mIntakeSubsystem;

  private final boolean mReverse;

  public RunIntakeMotors(IntakeSubsystem pIntakeSubsystem, boolean pReverse) {
    mIntakeSubsystem = pIntakeSubsystem;
    mReverse = pReverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSubsystem.setIntakeSpeedSetpoint(Constants.INTAKE_SPEED * (mReverse ? -1.0 : 1.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.setIntakeSpeedSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
