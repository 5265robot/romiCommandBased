// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnDeg extends CommandBase {
  private final RomiDrivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;

  /** Creates a new TurnDeg. */
  public TurnDeg(double speed, double degrees, RomiDrivetrain drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0,m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double inchPerDegree = Math.PI * 5.551 / 360;
    return getAverageTurningDistance() >= (inchPerDegree*m_degrees);
  }

  private double getAverageTurningDistance(){
    double leftDist = Math.abs(m_drive.getLeftDistanceInch());
    double rightDist = Math.abs(m_drive.getRightDistanceInch());
    return (leftDist + rightDist)/2.0;
  }
}
