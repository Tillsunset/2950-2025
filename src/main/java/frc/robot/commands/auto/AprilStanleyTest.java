package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilStanleyTest extends Command {
	private final driveTrain m_driveTrain;

	public AprilStanleyTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		m_driveTrain.resetPos();
		m_driveTrain.resetWaypoints();
	}

	@Override
	public void execute() {
		m_driveTrain.updateOdometry();
		// m_driveTrain.printPosVelHead();
		m_driveTrain.updateWaypoints();
		m_driveTrain.updateMotors();
		// m_driveTrain.printGoalMotor();
	}

	@Override
	public void end(boolean interrupted) {
        m_driveTrain.driveBase.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return m_driveTrain.getFinishedGoal();
	}
}
