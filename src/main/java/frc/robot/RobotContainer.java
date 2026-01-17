// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.alignment.AlignToPoleX;
import frc.robot.subsystems.vision.Limelight4Test;
import frc.robot.subsystems.vision.PhotonVisionGS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // // The robot's subsystems
  // private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

  // // The driver's controller
  // private final CommandXboxController driverController = new CommandXboxController(
  //     DRIVER_CONTROLLER_PORT);

  // // The operator's controller
  // private final CommandXboxController operatorController = new CommandXboxController(
  //     OPERATOR_CONTROLLER_PORT);

  // // The autonomous chooser
  // private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // /**
  //  * The container for the robot. Contains subsystems, OI devices, and commands.
  //  */
  // public RobotContainer() {
  //   configureBindings();

  //   // Set the options to show up in the Dashboard for selecting auto modes. If you
  //   // add additional auto modes you can add additional lines here with
  //   // autoChooser.addOption
  //   autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
  // }

  // /**
  //  * Use this method to define your trigger->command mappings. Triggers can be
  //  * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
  //  * constructor with an arbitrary predicate, or via the named factories in
  //  * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
  //  * for {@link CommandXboxController Xbox}/
  //  * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
  //  * controllers or
  //  * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
  //  * joysticks}.
  //  */
  // private void configureBindings() {

  //   // While the left bumper on operator controller is held, intake Fuel
  //   operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
  //   // While the right bumper on the operator controller is held, spin up for 1
  //   // second, then launch fuel. When the button is released, stop.
  //   operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
  //   // While the A button is held on the operator controller, eject fuel back out
  //   // the intake
  //   operatorController.a().whileTrue(new Eject(fuelSubsystem));

  //   // Set the default command for the drive subsystem to the command provided by
  //   // factory with the values provided by the joystick axes on the driver
  //   // controller. The Y axis of the controller is inverted so that pushing the
  //   // stick away from you (a negative value) drives the robot forwards (a positive
  //   // value)
  //   driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

  //   fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  // }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return autoChooser.getSelected();
  // }



  // The robot's subsystems
   private PhotonVisionGS m_vision = new PhotonVisionGS();
   private AlignToPoleX m_alignToPoleX = new AlignToPoleX();
   private Limelight4Test m_limelight4Test = new Limelight4Test();
   private DriveSubsystem m_robotDrive = new DriveSubsystem();//m_vision, m_alignToPoleX, m_limelight4Test, m_alignToPoleX);
  // The driver's controller
  private CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  // private CommandXboxController m_coDriver = new CommandXboxController(1);
  private AutoCommandManager m_autoManager = new AutoCommandManager( m_robotDrive);
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () ->
                  m_robotDrive.drive(
                      -m_driver.getLeftY()  * SwerveConstants.kMaxSpeedTeleop,
                      -m_driver.getLeftX()  * SwerveConstants.kMaxSpeedTeleop,
                      m_driver.getRightX() * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                      true),
              m_robotDrive));
    // m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    // SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));

    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.NONE))));

    // While the left bumper on operator controller is held, intake Fuel
    m_driver.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    m_driver.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    m_driver.a().whileTrue(new Eject(fuelSubsystem));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}