package frc.robot.commands;

import frc.robot.subsystems.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;

public class algeaArmDown extends Command {
	private final algaeArm m_Algearm;

	public algeaArmDown(algaeArm algearm){
		m_Algearm = algearm;
		addRequirements(m_Algearm);
	}

	@Override
	public void initialize() {
       // m_Algearm.updateTargetAngle(-50);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
