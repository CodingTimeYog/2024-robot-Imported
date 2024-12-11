package frc.robot.commands.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmToAngleCommand extends Command {
  private final ArmSubsystem mArmSubsystem;
  private final Angle mDesiredAngle;

  public SetArmToAngleCommand(ArmSubsystem pArmSubsystem, Angle pDesiredAngle) {
    mArmSubsystem = pArmSubsystem;
    mDesiredAngle = pDesiredAngle;

    addRequirements(mArmSubsystem);
  }

  @Override
  public void initialize() {
    mArmSubsystem.usePidController();
    mArmSubsystem.resetRateLimit();
  }

  @Override
  public void execute() {
    mArmSubsystem.setDesiredAngle(mDesiredAngle);
  }

  @Override
  public void end(boolean interrupted) {
    mArmSubsystem.disablePidController();
  }

  @Override
  public boolean isFinished() {
    return mArmSubsystem.atDesiredAngle();
  }
}
