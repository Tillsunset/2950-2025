package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorL2 extends Command {
	private final elevator m_elevator;

	public elevatorL2(elevator elevator) {
		m_elevator = elevator;
		addRequirements(m_elevator);
	}

	@Override
	public void initialize() {
		m_elevator.updateTargetPosition(17.5);
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
