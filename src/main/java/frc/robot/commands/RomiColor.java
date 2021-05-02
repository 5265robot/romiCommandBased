// Uses simple button presses to turn on the color LEDs of the Romi
// GRAMMAR!! -- char is single quotes; str is double quotes

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RomiColor extends CommandBase {
  
  // these are the lights on the Romi
  private DigitalOutput m_redLED; // = new DigitalOutput(3);
  private DigitalOutput m_greenLED; // = new DigitalOutput(1);

  // not sure why the subsystem had this on it
  private final char m_color;
  public enum ChannelMode{
    INPUT, OUTPUT
  }

  /** Creates a new RomiColor. */
  public RomiColor(char color) {
    m_color = color;  // we pass a single character: RYG to 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //OnBoardIO(ChannelMode.OUTPUT, ChannelMode.OUTPUT);
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

  // calling this creates a resource already allocated error
  // any new DigitalOutput channel creates the conflict
  public void OnBoardIO(ChannelMode dio1, ChannelMode dio2 ){
    if (dio1 != ChannelMode.INPUT) {
      m_greenLED = new DigitalOutput(1);
    }
    if (dio2 != ChannelMode.INPUT) {
      m_redLED = new DigitalOutput(2);
    }
  };

  private void setRed(boolean value){
    if (m_redLED != null) { 
      m_redLED.set(value);
    }
  }

  private void setGreen(boolean value){
    if (m_greenLED != null) { 
      m_greenLED.set(value);
    }
  }
}
