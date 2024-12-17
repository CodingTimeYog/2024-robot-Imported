package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.arm.MoveArmDownCommand;
import frc.robot.commands.arm.MoveArmToAmpCommand;
import frc.robot.commands.arm.RaiseArmCommand;
import frc.robot.commands.arm.SetArmToAngleCommand;
import frc.robot.commands.intake.GrabNoteCommand;
import frc.robot.commands.lifter.TogglePistonCommand;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.*;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer2 {

  // The robot's subsystems and commands are defined here...
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer2()
  {

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                driverXbox.getHID()::getYButtonPressed,
    //                                                                driverXbox.getHID()::getAButtonPressed,
    //                                                                driverXbox.getHID()::getXButtonPressed,
    //                                                                driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.5);
    // Configure the trigger bindings
    configureBindings();
    mCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", mCommandChooser);
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
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
        .onTrue(new InstantCommand(drivebase::zeroGyro, drivebase));
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

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return mCommandChooser.getSelected();
  }
}