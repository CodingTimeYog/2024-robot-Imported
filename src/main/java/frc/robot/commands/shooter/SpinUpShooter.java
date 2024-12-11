package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends Command {
  private final ShooterSubsystem mShooterSubsystem;

  public SpinUpShooter(ShooterSubsystem pShooterSubsystem) {
    mShooterSubsystem = pShooterSubsystem;
    addRequirements(mShooterSubsystem);
  }

  @Override
  public void execute() {
    mShooterSubsystem.setMotor(0.5);
  }

  @Override
  public boolean isFinished() {
    return mShooterSubsystem.isShooterUpToSpeed();
  }
}
