// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.autoByDist1;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.xbox;
import java.util.List;

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

  // for the auto choice on smart dashboard
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
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

    // smart dashboard for auto
    m_chooser.setDefaultOption("Trajectory 1", new StartEndCommand(
      () -> m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose()),
      () -> ramseteCommand.andThen(() -> m_romiDrivetrain.tankDriveVolts(0, 0)), 
      m_romiDrivetrain));
    }
      
    m_chooser.addOption("Distance 1", new autoByDist1(m_romiDrivetrain));

    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  public Command getArcadeDriveCommand(){
    return new ArcadeDrive(m_romiDrivetrain, 
    ()  -> -m_controller.getRawAxis(xbox.xBoxMove), 
    () -> m_controller.getRawAxis(xbox.xBoxTurn));
  }

  // Create a voltage constraint to ensure we don't accelerate too fast
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  // Create config for trajectory
  TrajectoryConfig config =
  new TrajectoryConfig(
           AutoConstants.kMaxSpeedMetersPerSecond,
           AutoConstants.kMaxAccelerationMetersPerSecondSquared)
       // Add kinematics to ensure max speed is actually obeyed
       .setKinematics(DriveConstants.kDriveKinematics)
       // Apply the voltage constraint
       .addConstraint(autoVoltageConstraint);
  // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config);
  // example Ramsete command
  RamseteCommand ramseteCommand =
  new RamseteCommand(
      exampleTrajectory,
      m_romiDrivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_romiDrivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_romiDrivetrain::tankDriveVolts,
      m_romiDrivetrain);

}
