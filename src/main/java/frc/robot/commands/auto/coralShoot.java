package frc.robot.commands.auto;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class coralShoot extends Command {
	private final coralIntake m_coralIntake;

	private Timer m_timer;
	
	public coralShoot(coralIntake coralIntake) {
		m_coralIntake = coralIntake;
		addRequirements(m_coralIntake);
		m_timer = new Timer();
 	}
 
	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		m_coralIntake.setOutput(-1);
	}

	@Override
	public void end(boolean interrupted) {
		m_coralIntake.setOutput(0);
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 1;
	}
}
