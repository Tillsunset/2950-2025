package frc.robot.commands;

import frc.robot.subsystems.driveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class driveTank extends Command {
	private final driveTrain m_driveTrain;
	private DoubleSupplier leftAxis;
	private DoubleSupplier rightAxis;
	private DoubleSupplier flipFront;
	private double scale = -1;

	public driveTank(driveTrain driveTrain, XboxController x) {
		leftAxis = x::getLeftY;
		rightAxis = x::getRightY;
		flipFront = x::getLeftTriggerAxis;
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (Math.abs(flipFront.getAsDouble()) > 0.1) {
			m_driveTrain.driveBase.tankDrive(-scale * rightAxis.getAsDouble(), -scale * leftAxis.getAsDouble());

		}
		else {
			m_driveTrain.driveBase.tankDrive(scale * leftAxis.getAsDouble(), scale * rightAxis.getAsDouble());
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
