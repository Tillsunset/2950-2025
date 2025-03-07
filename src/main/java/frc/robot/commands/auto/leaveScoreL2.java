package frc.robot.commands.auto;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class leaveScoreL2 extends Command {
	private final driveTrain m_driveTrain;

	private Timer m_timer;  
	
	public leaveScoreL2(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
		m_timer = new Timer();
 	}
 
	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		m_driveTrain.driveBase.tankDrive(-0.4,-0.4);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveTrain.driveBase.stopMotor();
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 2;
	}
}
