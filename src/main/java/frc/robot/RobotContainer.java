// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SignalControl;
import frc.robot.commands.Swerve_drive_joystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Swerve_subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //joystick and button
  public static Joystick drive = new Joystick(0);
  public static JoystickButton drive_A = new JoystickButton(drive, 1);
  public static Joystick stick = new Joystick(Constants.joystickport);
  //motor command and subsystem
  // public static intake m_intake = new intake(Constants.intake_port);
  // public static arm Arm = new arm(Constants.arm_port);
  public static SignalControl m_signalcontrol = new SignalControl(/*m_intake,Arm*/);
  public static Swerve_subsystem m_calculate = new Swerve_subsystem(); 
  public static SwerveModule LeftFrontSwerveModule = new SwerveModule(1, 34, 14, 3601, 0);      //(number, angleMotorPort, velocityMotorPort, zeroPosition)
  public static SwerveModule RightFrontSwerveModule = new SwerveModule(2, 32, 12, 3831, 1);
  public static SwerveModule RightBackSwerveModule = new SwerveModule(3, 38, 18, 754, 1);
  public static SwerveModule LeftBackSwerveModule = new SwerveModule(4, 36, 16, 3491, 0);
  public static Swerve_drive_joystick m_swerve_drive = new Swerve_drive_joystick(m_calculate);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
