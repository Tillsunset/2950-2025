package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorL2 extends Command {
	private final elevator m_elevator;
	private Timer m_timer;

	public elevatorL2(elevator elevator) {
		m_elevator = elevator;
		addRequirements(m_elevator);
	}

	@Override
	public void initialize() {
		m_elevator.l2();
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 0.5;
	}
}
