// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the Romi gyroscope
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the accelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Odometry for tracking the robot pose
  private final DifferentialDriveOdometry m_odometry;

  // field diagram ??
  private final Field2d m_field2d = new Field2d();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    
    // to get this to work, I had to implement Gyro in RomiGyro
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // explain this please
    SmartDashboard.putData("field", m_field2d);
  }

  /*
   * arcade drive
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  /*
   * tank drive
   * individual values for left and right motors
   * negative right voltage so that plus is always forward
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  /* 
   * encoder commands
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public int getLeftEncoderCount(){
    return m_leftEncoder.get();
  }
  public int getRightEncoderCount(){
    return m_rightEncoder.get();
  }
  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }
  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }
  public double getAverageDistanceInch(){
    return (getLeftDistanceInch() + getRightDistanceInch())/2.0;
  }
  /*
   * acceleration about an axis
   */
  public double getAccelX(){
    return m_accelerometer.getX();
  }
  public double getAccelY(){
    return m_accelerometer.getY();
  }
  public double getAccelZ(){
    return m_accelerometer.getZ();
  }
  /*
   * current angle of the Romi around each axis
   */
  public double getGyroAngleX(){
    return m_gyro.getAngleX();
  }
  public double getGyroAngleY(){
    return m_gyro.getAngleY();
  }  
  public double getGyroAngleZ(){
    return m_gyro.getAngleZ();
  }
  public void resetGyro(){
    m_gyro.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // same as periodic ??
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    // update the field
    m_field2d.setRobotPose(getPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // update the odometry
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    // update the field
    m_field2d.setRobotPose(getPose());
  }

  // returns the current estimated Pose
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  // returns current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate());
  }

  // resets odometry to a pose
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  // set max speed, which slows down the robot
  public void setMaxOutput(double maxOutput){
    m_diffDrive.setMaxOutput(maxOutput);
  }

  // zeroes the heading
  public void zeroHeading(){
    m_gyro.reset();
  }

  // returns the current heading
  public double getHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }

  // returns the current turn rate
  public double getTurnRate(){
    return m_gyro.getRate();
  }

  // dif drive wheel speeds
  // reset odometry
  // set max output
  // zero heading
  // get heading
  // get turn rate

}
