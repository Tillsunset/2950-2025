package frc.robot.commands;

import frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;

public class algeaOut extends Command {
	private final algaeIntake m_algeaIntake;

	public algeaOut(algaeIntake algeaIntake){
		m_algeaIntake = algeaIntake;
		addRequirements(m_algeaIntake);
	}

	@Override
	public void initialize() {
		m_algeaIntake.setOutput(-0.5);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		m_algeaIntake.setOutput(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
