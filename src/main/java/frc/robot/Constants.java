// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
    
        public static final double kPDriveVel = 0.085;
    
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
      }

      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

    public static final class xbox{
        public static int xBoxMove = 1;
        public static int xBoxTurn = 4;
        public static XboxController.Button xBoxGreen = Button.kA;
        public static XboxController.Button xBoxYellow = Button.kY;
        public static XboxController.Button xBoxRed = Button.kB;
    }

    public static final class autoD1{
        // first distance -- speed then distance in inches
        // second turn -- speed then angle from 0 to 360
        // positive speed is forward
        // distance is ALWAYS positive
        // angle is ALWAYS positive
        public static double dstA_sp = 0.5, dstA_dst = 10.0;
        public static double trnA_sp = -0.5, trnA_ang = 90.0;
        public static double dstB_sp = -0.5, dstB_dst = 10.0;
        public static double trnB_sp = 0.5, trnB_ang = 90.0;
    }


}
