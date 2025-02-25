package frc.robot.commands;

import frc.robot.subsystems.winch;

import edu.wpi.first.wpilibj2.command.Command;

public class winchUp extends Command {
	private final winch m_winch;

	public winchUp(winch winch){
		m_winch = winch;
		addRequirements(m_winch);
	}

	@Override
	public void initialize() {
        m_winch.winch.set(1.0);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
        m_winch.winch.set(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
