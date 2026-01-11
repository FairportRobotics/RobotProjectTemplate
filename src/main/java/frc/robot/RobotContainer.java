// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive.SwerveBuilder;
import org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive.SwerveDriveSubsystem;

import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveDriveSubsystem m_SwerveDriveSubsystem;

  private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser.addOption("Test1", Commands.run(new Runnable() {
      @Override
      public void run() {
        System.out.println("Command 1");
      }
      
    }, m_exampleSubsystem));
    autoChooser.addOption("Test2", Commands.run(new Runnable() {
      @Override
      public void run() {
        System.out.print("Command 2");
      }
      
    }, m_exampleSubsystem));

    SmartDashboard.putData(autoChooser);

    SwerveBuilder swerveBuilder = new SwerveBuilder();
    m_SwerveDriveSubsystem = swerveBuilder
        .withCanbusName("null")
        .withPigeonId(1) 
        .withSwerveModule(swerveBuilder.new SwerveModuleBuilder()
                          .withDriveMotorId(2)
                          .withDriveKP(1)
                          .withDriveKI(0)
                          .withDriveKD(0)
                          .withSteerMotorId(3)
                          .withSteerKP(1)
                          .withSteerKI(0)
                          .withSteerKD(0)
                          .withSteerEncoderId(1)
                          .withGearRatio(1)
                          .withModuleLocation(new Translation2d(-11.5, 11.5))
                          .withModuleName("Front Left")
                          .build())
        .withSwerveModule(swerveBuilder.new SwerveModuleBuilder()
                          .withDriveMotorId(5)
                          .withDriveKP(1)
                          .withDriveKI(0)
                          .withDriveKD(0)
                          .withSteerMotorId(6)
                          .withSteerKP(1)
                          .withSteerKI(0)
                          .withSteerKD(0)
                          .withSteerEncoderId(4)
                          .withGearRatio(1)
                          .withModuleLocation(new Translation2d(11.5, 11.5))
                          .withModuleName("Front Right")
                          .build())
        .withSwerveModule(swerveBuilder.new SwerveModuleBuilder()
                          .withDriveMotorId(11)
                          .withDriveKP(1)
                          .withDriveKI(0)
                          .withDriveKD(0)
                          .withSteerMotorId(12)
                          .withSteerKP(1)
                          .withSteerKI(0)
                          .withSteerKD(0)
                          .withSteerEncoderId(10)
                          .withGearRatio(1)
                          .withModuleLocation(new Translation2d(11.5, -11.5))
                          .withModuleName("Back Right")
                          .build())
        .withSwerveModule(swerveBuilder.new SwerveModuleBuilder()
                          .withDriveMotorId(8)
                          .withDriveKP(1)
                          .withDriveKI(0)
                          .withDriveKD(0)
                          .withSteerMotorId(9)
                          .withSteerKP(1)
                          .withSteerKI(0)
                          .withSteerKD(0)
                          .withSteerEncoderId(7)
                          .withGearRatio(1)
                          .withModuleLocation(new Translation2d(-11.5, -11.5))
                          .withModuleName("Back Left")
                          .build())
        .build();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public Command getDriveCommand() {
    return Commands.run(new Runnable() {
      @Override
      public void run() {
        m_SwerveDriveSubsystem.setChassisSpeedsFromJoystick(m_driverController.getLeftX(), m_driverController.getLeftY(), m_driverController.getRightX());
      } 
    }, m_SwerveDriveSubsystem);
  }
}
