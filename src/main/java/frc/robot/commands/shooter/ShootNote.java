package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNote extends Command {
  private final ShooterSubsystem mShooter;
  private final LoaderSubsystem mLoader;

  public ShootNote(ShooterSubsystem pShooter, LoaderSubsystem pLoader) {
    mShooter = pShooter;
    mLoader = pLoader;

    addRequirements(pLoader, pShooter);
  }

  @Override
  public void execute() {
    var shooterSpeed = SmartDashboard.getNumber("ShootNote: Shooter Speed", 0.5);
    mShooter.setMotor(shooterSpeed);

    if (mShooter.isShooterUpToSpeed()) {
      mLoader.setDesiredLoaderSpeed(Constants.LOADER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    mLoader.setDesiredLoaderSpeed(0);
    mShooter.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return !mLoader.hasNote();
  }
}
