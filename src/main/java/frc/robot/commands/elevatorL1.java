package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorL1 extends Command {
	private final elevator m_elevator;

	public elevatorL1(elevator elevator) {
		m_elevator = elevator;
		addRequirements(m_elevator);
	}

	@Override
	public void initialize() {
		m_elevator.l1();
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
