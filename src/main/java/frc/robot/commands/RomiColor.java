// Uses simple button presses to turn on the color LEDs of the Romi
// GRAMMAR!! -- char is single quotes; str is double quotes

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RomiColor extends CommandBase {
  private final char m_color;

  private final DigitalOutput m_redLED = new DigitalOutput(2);
  //private final DigitalOutput m_greenLED = new DigitalOutput(1);


  /** Creates a new RomiColor. */
  public RomiColor(char color) {
    m_color = color;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_color == 'Y') {
      setRed(false);
      setGreen(false);
    }
    else if (m_color == 'G') {
      setGreen(true);
    }
    else if (m_color == 'R') {
      setRed(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean setRed(boolean value){
    /*if (m_redLED != null) { 
      m_redLED.set(value);
    }*/
    return true;
  }

  private boolean setGreen(boolean value){
    /*if (m_greenLED != null) { 
      m_greenLED.set(value);
    }*/
    return true;
  }
}
