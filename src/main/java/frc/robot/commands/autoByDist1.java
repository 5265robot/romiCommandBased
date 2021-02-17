// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.autoD1;
import frc.robot.subsystems.RomiDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoByDist1 extends SequentialCommandGroup {
  /** Creates a new autoByDistance. */
  public autoByDist1(RomiDrivetrain drivetrain) {
    addCommands(
      new DriveDist(autoD1.dstA_sp, autoD1.dstA_dst, drivetrain),
      new TurnDeg(autoD1.trnA_sp, autoD1.trnA_ang, drivetrain),
      new DriveDist(autoD1.dstB_sp, autoD1.dstB_dst, drivetrain),
      new TurnDeg(autoD1.trnB_sp, autoD1.trnB_ang, drivetrain)
    );
  }
}
