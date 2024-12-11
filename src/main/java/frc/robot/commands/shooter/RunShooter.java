// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class RunShooter extends Command {
  private final ShooterSubsystem mShooterSubsystem;
  private final DoubleSupplier mSpeedSupplier;

  /** Creates a new ShootCommand. */
  public RunShooter(ShooterSubsystem pShooterSubsystem, DoubleSupplier pSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mShooterSubsystem = pShooterSubsystem;
    this.mSpeedSupplier = pSpeedSupplier;

    addRequirements(mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooterSubsystem.setMotor(mSpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooterSubsystem.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
