// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTest1;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.xbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final XboxController m_controller = new XboxController(0);


  private final AutoTest1 m_autoCommand = new AutoTest1(m_romiDrivetrain);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(getArcadeDriveCommand());

    // examples of xbox buttons turning on and off the LEDs of the Romi
    // this does not work - hasn't been figured out yet
    /*new JoystickButton(m_controller, xbox.xBoxRed.value).whenPressed(new RomiColor('R'));
    new JoystickButton(m_controller, xbox.xBoxGreen.value).whenPressed(new RomiColor('G'));
    new JoystickButton(m_controller, xbox.xBoxYellow.value).whenPressed(new RomiColor('Y'));
    */



    //(m_controller.getRawButtonPressed(Constants.xBoxYellow));
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

  public Command getArcadeDriveCommand(){
    return new ArcadeDrive(m_romiDrivetrain, 
    ()  -> -m_controller.getRawAxis(xbox.xBoxMove), 
    () -> m_controller.getRawAxis(xbox.xBoxTurn));
  }
}
