package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmDownCommand extends Command {
  private final ArmSubsystem mArmSubsystem;

  public MoveArmDownCommand(ArmSubsystem pArmSubsystem) {
    mArmSubsystem = pArmSubsystem;
    addRequirements(mArmSubsystem);
  }

  @Override
  public void initialize() {
    mArmSubsystem.disablePidController();
  }

  @Override
  public void execute() {
    mArmSubsystem.setValue(-0.35);
  }

  @Override
  public boolean isFinished() {
    return mArmSubsystem.atBottom();
  }

  @Override
  public void end(boolean interrupted) {
    mArmSubsystem.setValue(0.0);
  }
}
