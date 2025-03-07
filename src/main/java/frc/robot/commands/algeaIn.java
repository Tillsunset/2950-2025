package frc.robot.commands;

import frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;

public class algeaIn extends Command {
	private final algaeIntake m_algeaIntake;

	public algeaIn(algaeIntake algeaIntake){
		m_algeaIntake = algeaIntake;
		addRequirements(m_algeaIntake);
	}

	@Override
	public void initialize() {
		m_algeaIntake.setOutput(1.);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		m_algeaIntake.setOutput(0.1);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
