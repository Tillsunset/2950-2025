package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorL2 extends Command {
	private final elevator m_elevator;

	/** Creates a new elevatorUp. */
	public elevatorL2(elevator elevator) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_elevator = elevator;
		addRequirements(m_elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_elevator.updateTargetPosition(20);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
