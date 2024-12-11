package frc.robot.commands.lifter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class TogglePistonCommand extends Command {
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final boolean extend;
  private final ArmSubsystem armSubsystem;

  public TogglePistonCommand(
      PneumaticsSubsystem pneumaticsSubsystem, boolean extend, ArmSubsystem armSubsystem) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.extend = extend;
    this.armSubsystem = armSubsystem;

    addRequirements(pneumaticsSubsystem);
    addRequirements(armSubsystem);
  }

  //
  @Override
  public void execute() {
    if (extend) {
      if (!pneumaticsSubsystem.isPistonFullyExtended()) {
        pneumaticsSubsystem.extendPiston();
      } else {
        pneumaticsSubsystem.stopPiston();
      }
    } else {
      if (!pneumaticsSubsystem.isPistonFullyRetracted()) {
        pneumaticsSubsystem.retractPiston();
      } else {
        pneumaticsSubsystem.stopPiston();
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (extend) {
      if (pneumaticsSubsystem.isPistonFullyExtended()) {
        System.out.println("isFinished fully extended");
        finished = true;
      }
    } else {
      if (pneumaticsSubsystem.isPistonFullyRetracted()) {
        System.out.println("isFinished fully retracted");
        finished = true;
      }
    }
    System.out.println("inFinished: " + finished);
    return finished;
  }
}
