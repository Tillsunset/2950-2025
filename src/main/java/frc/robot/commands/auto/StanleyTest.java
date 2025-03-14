package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import frc.robot.Pose;

import java.util.ArrayList;
import java.util.Currency;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class StanleyTest extends Command {
	private final driveTrain m_driveTrain;

	public StanleyTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		m_driveTrain.resetPos();
		m_driveTrain.resetWaypoints();
		m_driveTrain.goal.x = 1;
		m_driveTrain.goal.y = 0;
		m_driveTrain.interpolateAlignedPoints(m_driveTrain.current, m_driveTrain.goal, m_driveTrain.numPoints);
	}

	@Override
	public void execute() {

		m_driveTrain.updatePos();
		// m_driveTrain.printPosVelHead();
		m_driveTrain.updateMotors();
	}

	@Override
	public void end(boolean interrupted) {
        m_driveTrain.driveBase.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return m_driveTrain.getGoalDistance() < 0.2;
	}
}
