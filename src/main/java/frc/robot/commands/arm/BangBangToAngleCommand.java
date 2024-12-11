package frc.robot.commands.arm;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class BangBangToAngleCommand extends Command {
  private final ArmSubsystem mArmSubsystem;
  private final BangBangController mBangBangController;
  private final Angle mDesiredAngle;

  public BangBangToAngleCommand(ArmSubsystem pArmSubsystem, Angle pDesiredAngle) {
    mArmSubsystem = pArmSubsystem;
    mDesiredAngle = pDesiredAngle;
    addRequirements(pArmSubsystem);

    mBangBangController = new BangBangController(0.25);
  }

  @Override
  public void execute() {
    var measurement = mArmSubsystem.getArmPosition().in(Units.Degrees);
    var output = mBangBangController.calculate(measurement, mDesiredAngle.in(Units.Degrees));

    mArmSubsystem.setValue(output * 0.35);
  }

  @Override
  public void initialize() {
    mArmSubsystem.enableBangBang();
  }

  @Override
  public void end(boolean interrupted) {
    mArmSubsystem.disableBangBang();
  }

  @Override
  public boolean isFinished() {
    return mBangBangController.atSetpoint();
  }
}
