// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.interfaces.Gyro;


// implements Gyro added by GN
public class RomiGyro implements Gyro{
  private SimDouble m_simRateX,
    m_simRateY,
    m_simRateZ,
    m_simAngleX,
    m_simAngleY,
    m_simAngleZ;

  private double m_angleXOffset,
    m_angleYOffset,
    m_angleZOffset;

  // moved instance to outside constructor
  private SimDevice gyroSimDevice;

  /** Creates a new RomiGyro. */
  public RomiGyro() {
    gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (gyroSimDevice != null) {
      gyroSimDevice.createBoolean("init", Direction.kOutput, true);

      m_simRateX = gyroSimDevice.createDouble("rate_x", Direction.kInput, 0.0);
      m_simRateY = gyroSimDevice.createDouble("rate_y", Direction.kInput, 0.0);
      m_simRateZ = gyroSimDevice.createDouble("rate_z", Direction.kInput, 0.0);
      
      m_simAngleX = gyroSimDevice.createDouble("angle_x", Direction.kInput, 0.0);
      m_simAngleY = gyroSimDevice.createDouble("angle_y", Direction.kInput, 0.0);
      m_simAngleZ = gyroSimDevice.createDouble("anlge_z", Direction.kInput, 0.0);   
    }
  }
  /*
  * Return the rate of turn in degrees-per-second around each axis
  */
  public double getRateX() {
    if (m_simRateX != null){
      return m_simRateX.get();
    }
    return 0.0;
  }
  public double getRateY() {
    if (m_simRateY != null){
      return m_simRateY.get();
    }
    return 0.0;
  }
  public double getRateZ() {
    if (m_simRateZ != null){
      return m_simRateZ.get();
    }
    return 0.0;
  }
  
  /*
   * Return currently reported angle around each axis 
   */
  public double getAngleX(){
    if (m_simAngleX != null) {
      return m_simAngleX.get() - m_angleXOffset;
    }
    return 0.0;
  }
  public double getAngleY(){
    if (m_simAngleY != null) {
      return m_simAngleY.get() - m_angleYOffset;
    }
    return 0.0;
  }
  public double getAngleZ(){
    if (m_simAngleZ != null) {
      return m_simAngleZ.get() - m_angleZOffset;
    }
    return 0.0;
  }

  // Reset the gyro angles to 0
  public void reset(){
    if (m_simAngleX != null){
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    }
  }

  @Override
  public void close() throws Exception {
    // created GN
    if (gyroSimDevice != null) {
      gyroSimDevice.close();
    }
  }

  @Override
  public void calibrate() {
    // created GN
    // no-op
  }

  @Override
  public double getAngle() {
    // created GN
    return getAngleZ();
  }

  @Override
  public double getRate() {
    // created GN
    return getRateZ();
  }

}
