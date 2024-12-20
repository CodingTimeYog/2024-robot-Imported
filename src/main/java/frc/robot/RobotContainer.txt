// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntakeMotors;
import frc.robot.commands.RunLoaderMotor;
import frc.robot.commands.arm.MoveArmDownCommand;
import frc.robot.commands.arm.MoveArmToAmpCommand;
import frc.robot.commands.arm.RaiseArmCommand;
import frc.robot.commands.arm.SetArmToAngleCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.intake.GrabNoteCommand;
import frc.robot.commands.lifter.TogglePistonCommand;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  
  private final Drivetrain_Subsystem_Second drivebase = new Drivetrain_Subsystem_Second(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final CommandXboxController mController =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController mOperatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_ID);

  private final PneumaticsSubsystem mLifterSubsystem = new PneumaticsSubsystem();

  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

  private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

  private final LoaderSubsystem mLoaderSubsystem = new LoaderSubsystem();
  private final LedSubsystem mLedSubsystem = new LedSubsystem();

  private final SendableChooser<Command> mCommandChooser;

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    mDrivetrainSubsystem.setChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d()));
    mDrivetrainSubsystem.zeroGyroscope();

    mDrivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            mDrivetrainSubsystem,
            () ->
                -modifyAxis(mController.getLeftY())
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(mController.getLeftX())
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(mController.getRightX())
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                    * 0.5));

    // Configure the button bindings
    configureButtonBindings();

    mCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", mCommandChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    NamedCommands.registerCommand(
        "grab_note", new GrabNoteCommand(mLoaderSubsystem, mIntakeSubsystem));
    NamedCommands.registerCommand("shoot_note", new ShootNote(mShooterSubsystem, mLoaderSubsystem));
    NamedCommands.registerCommand("move_arm_down", new MoveArmDownCommand(mArmSubsystem));
    NamedCommands.registerCommand(
        "move_arm_to_amp_position", new MoveArmToAmpCommand(mArmSubsystem));
    NamedCommands.registerCommand(
        "move_arm_to_speaker_position",
        new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(10.0)));

    var hasNoteTrigger = new Trigger(mLoaderSubsystem::hasNote);
    var inPositionToShootTrigger =
        new Trigger(() -> mArmSubsystem.getArmPosition().isNear(Units.Degrees.of(10.0), 0.25));

    hasNoteTrigger.whileTrue(mLedSubsystem.writeStaticColor(0, 255, 0, 1.0));
    inPositionToShootTrigger.whileTrue(mLedSubsystem.colorFadeCommand(255, 255, 255));
    // hasNoteTrigger.whileFalse(mLedSubsystem.strobeColor(255, 0, 0));

    mController
        .a()
        .onTrue(new InstantCommand(mDrivetrainSubsystem::zeroGyroscope, mDrivetrainSubsystem));
    mController
        .rightTrigger(0.75)
        .or(
            mOperatorController
                .a()
                .and(mOperatorController.leftBumper().negate())
                .and(mOperatorController.rightBumper().negate()))
        .whileTrue(new ShootNote(mShooterSubsystem, mLoaderSubsystem));

    mOperatorController
        .y()
        .and(mOperatorController.leftBumper())
        .whileTrue(new RunLoaderMotor(mLoaderSubsystem, true));
    mOperatorController
        .b()
        .and(mOperatorController.leftBumper().negate())
        .whileTrue(new RunIntakeMotors(mIntakeSubsystem, false));
    mOperatorController
        .b()
        .and(mOperatorController.leftBumper())
        .whileTrue(new RunIntakeMotors(mIntakeSubsystem, true));
    mOperatorController.x().whileTrue(new GrabNoteCommand(mLoaderSubsystem, mIntakeSubsystem));
    mOperatorController
        .y()
        .and(mOperatorController.leftBumper().negate())
        .whileTrue(new RunLoaderMotor(mLoaderSubsystem, false));

    mOperatorController
        .a()
        .and(mOperatorController.rightBumper())
        .whileTrue(new RunShooter(mShooterSubsystem, () -> 0.5));
    mOperatorController
        .a()
        .and(mOperatorController.leftBumper())
        .whileTrue(new RunShooter(mShooterSubsystem, () -> -0.1));

    mOperatorController
        .leftBumper()
        .whileTrue(new RaiseArmCommand(mArmSubsystem, () -> mOperatorController.getLeftY() * -0.6));
    // Original speed -0.45

    mOperatorController
        .povUp()
        .whileTrue(new TogglePistonCommand(mLifterSubsystem, true, mArmSubsystem));
    mOperatorController
        .povDown()
        .whileTrue(new TogglePistonCommand(mLifterSubsystem, false, mArmSubsystem));

    // mOperatorController
    // .start().whileTrue(new BangBangToAngleCommand(mArmSubsystem,
    // Units.Degrees.of(5.0)));
    // mOperatorController.start().whileTrue(new SetArmToAngleCommand(mArmSubsystem,
    // Units.Degrees.of(10.0)));
    mOperatorController
        .rightTrigger(0.5)
        .whileTrue(new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(10.0)));
    // mOperatorController
    //     .leftTrigger(0.5)
    //     .whileTrue(new SetArmToAngleCommand(mArmSubsystem, Units.Degrees.of(30)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mCommandChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05); // sanjay change: Deadband value before 0.05

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
