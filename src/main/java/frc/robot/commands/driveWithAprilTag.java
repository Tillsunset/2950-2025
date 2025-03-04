package frc.robot.commands;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class driveWithAprilTag extends Command {
	private final driveTrain m_driveTrain;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	double forwardGoalDistance = 2;
	double sideGoalDistance = 3;

	double kPDistance = 0.1;
	double kPFacing = 0.01;

	// distance from the center of the Limelight lens to the floor
	double limelightLensHeightMeters = 0.5;
	double goalHeightMeters = 1.5;

	// how many degrees back is your limelight rotated from perfectly vertical?
	double limelightMountAngleDegrees = 25.0;

	public driveWithAprilTag(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	@Override
	public void initialize() {
	}

	enum t2dEnum {
		tx(0),
		ty(1),
		tz(2),
		pitch(3),
		yaw(4),
		roll(5);

		private final int val;

		private t2dEnum(int val) { 
			this.val = val; 
		} 
	}

	@Override
	public void execute() {
		double[] t2d = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
		if (t2d.length == 6) {
			double xOffset = t2d[t2dEnum.tx.val];
			double xScale = -2;

			double distance = t2d[t2dEnum.ty.val];
			double distanceError = distance - forwardGoalDistance;
			double distanceSCale = 0.0;

			m_driveTrain.driveBase.arcadeDrive(distanceError * distanceSCale, xOffset * xScale, false);
		}
		else {
			m_driveTrain.driveBase.feed();
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
