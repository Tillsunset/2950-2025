package frc.robot.commands;

import frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;

public class coralIn extends Command {
	private final coralIntake m_coralIntake;

	public coralIn(coralIntake coralintake){
		m_coralIntake = coralintake;
		addRequirements(m_coralIntake);
	}

	@Override
	public void initialize() {
		m_coralIntake.setOutput(1.);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		m_coralIntake.setOutput(0.1);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
