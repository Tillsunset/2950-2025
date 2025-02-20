package frc.robot.commands;

import frc.robot.subsystems.winch;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * An example command that uses an example subsystem.
 */
public class winchUp extends Command {
	private final winch m_winch;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public winchUp(winch winch){
		// Use requires() here to declare subsystem dependencies.
		m_winch = winch;
		addRequirements(m_winch);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        m_winch.winch.set(1);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        m_winch.winch.set(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
