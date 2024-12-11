package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;

/**
 * Attempts to grab a note.
 *
 * <p>Finishes when the loader subsystem says it has a note.
 */
public class GrabNoteCommand extends Command {

  private LoaderSubsystem mLoaderSubsystem;
  private IntakeSubsystem mIntakeSubsystem;

  public GrabNoteCommand(LoaderSubsystem pLoaderSubsystem, IntakeSubsystem pIntakeSubsystem) {
    mLoaderSubsystem = pLoaderSubsystem;
    mIntakeSubsystem = pIntakeSubsystem;

    addRequirements(mLoaderSubsystem, mIntakeSubsystem);
  }

  @Override
  public void execute() {
    var intakeSpeed = SmartDashboard.getNumber("GrabNote: Intake Speed", Constants.INTAKE_SPEED);
    var loaderSpeed = SmartDashboard.getNumber("GrabNote: Loader Speed", Constants.LOADER_SPEED);

    mIntakeSubsystem.setIntakeSpeedSetpoint(intakeSpeed);
    mLoaderSubsystem.setDesiredLoaderSpeed(loaderSpeed);
  }

  @Override
  public boolean isFinished() {
    return mLoaderSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.setIntakeSpeedSetpoint(0.0);
    mLoaderSubsystem.stopLoaderMotors();
  }
}
