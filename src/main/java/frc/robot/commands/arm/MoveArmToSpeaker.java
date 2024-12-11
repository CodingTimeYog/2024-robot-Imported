package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToSpeaker extends Command {
  private final ArmSubsystem mArmSubsystem;

  public MoveArmToSpeaker(ArmSubsystem pArmSubsystem) {
    mArmSubsystem = pArmSubsystem;
    addRequirements(pArmSubsystem);
  }

  @Override
  public void initialize() {
    mArmSubsystem.disablePidController();
    mArmSubsystem.resetRateLimit();
  }

  @Override
  public void execute() {}
}
