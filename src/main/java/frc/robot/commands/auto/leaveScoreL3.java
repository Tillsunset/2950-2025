package frc.robot.commands.auto;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class leaveScoreL3 extends Command {
	private final driveTrain m_driveTrain;
	private final elevator m_elevator;
	private final coralIntake m_coralIntake;

	private Timer m_timer;  
	
	public leaveScoreL3(driveTrain driveTrain, elevator elevator, coralIntake coralIntake) {
		m_driveTrain = driveTrain;
		m_elevator = elevator;
		m_coralIntake = coralIntake;
		addRequirements(m_driveTrain);
		addRequirements(m_elevator);
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
		if (m_timer.get() < 4) {
			m_driveTrain.driveBase.tankDrive(-0.35,-0.35);
		}
		else if (m_timer.get() < 5) {
			m_driveTrain.driveBase.stopMotor();
			m_elevator.l3();
		}
		else if (m_timer.get() < 6) {
			m_coralIntake.setOutput(-1.);
		}

	}

	@Override
	public void end(boolean interrupted) {
		m_driveTrain.driveBase.stopMotor();
		m_elevator.l1();
		m_coralIntake.setOutput(0);
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 8;
	}
}
