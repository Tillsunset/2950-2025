package frc.robot.commands;

import frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;

public class coralOut extends Command {
	private final coralIntake m_coralIntake;

	public coralOut(coralIntake coralintake){
		m_coralIntake = coralintake;
		addRequirements(m_coralIntake);
	}

	@Override
	public void initialize() {
		m_coralIntake.setOutput(-1);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		m_coralIntake.setOutput(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
