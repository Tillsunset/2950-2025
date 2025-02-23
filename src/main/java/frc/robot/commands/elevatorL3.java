package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorL3 extends Command {
	private final elevator m_elevator;

	public elevatorL3(elevator elevator) {
		m_elevator = elevator;
		addRequirements(m_elevator);
	}

	@Override
	public void initialize() {
		m_elevator.updateTargetPosition(47);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
