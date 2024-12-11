// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class RaiseArmCommand extends Command {

  private final ArmSubsystem mArmSubsystem;
  private final DoubleSupplier mPositionSupplier;
  private final SlewRateLimiter mRateLimiter;

  /** Creates a new raiseArm. */
  public RaiseArmCommand(ArmSubsystem pArmSubsystem, DoubleSupplier pPositionSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArmSubsystem = pArmSubsystem;
    mPositionSupplier = pPositionSupplier;
    mRateLimiter = new SlewRateLimiter(0.5);
    addRequirements(mArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArmSubsystem.disablePidController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = mRateLimiter.calculate(mPositionSupplier.getAsDouble());
    SmartDashboard.putNumber("RaiseArmCommand: Arm Setpoint", setpoint);

    mArmSubsystem.setValue(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArmSubsystem.disablePidController();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
