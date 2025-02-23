package frc.robot.commands;

import frc.robot.subsystems.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;

public class algeaArmUp extends Command {
	private final algaeArm m_Algearm;

	public algeaArmUp(algaeArm algearm){
		m_Algearm = algearm;
		addRequirements(m_Algearm);
	}

	@Override
	public void initialize() {
       // m_Algearm.updateTargetAngle(0);
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
