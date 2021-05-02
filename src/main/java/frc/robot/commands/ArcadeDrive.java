package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase{
    private final RomiDrivetrain m_drivetrain;
    private final Supplier<Double> m_xAxisValue;
    private final Supplier<Double> m_zAxisValue;

    public ArcadeDrive (
            RomiDrivetrain drivetrain, 
            Supplier<Double> xAxis , 
            Supplier<Double> zAxis){
        m_drivetrain = drivetrain;
        m_xAxisValue = xAxis;
        m_zAxisValue = zAxis;
        addRequirements(drivetrain);
    }

	// Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(m_xAxisValue.get(), m_zAxisValue.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
