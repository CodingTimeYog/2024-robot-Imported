package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
  private final DrivetrainSubsystem mDrivetrain;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  private final SlewRateLimiter mRateLimiterX = new SlewRateLimiter(2);
  private final SlewRateLimiter mRateLimiterY = new SlewRateLimiter(2);

  public DefaultDriveCommand(
      DrivetrainSubsystem pDrivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.mDrivetrain = pDrivetrain;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(pDrivetrain);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented movement
    var desiredTranslationX = m_translationXSupplier.getAsDouble();
    var desiredTranslationY = m_translationYSupplier.getAsDouble();
    var desiredRotation = m_rotationSupplier.getAsDouble();
    var fieldRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredTranslationX,
            desiredTranslationY,
            desiredRotation,
            mDrivetrain.getGyroscopeRotation());

    mDrivetrain.setChassisSpeeds(fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrain.setChassisSpeedsRobotRelative(new ChassisSpeeds());
  }
}
