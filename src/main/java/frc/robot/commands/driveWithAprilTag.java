/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An example command that uses an example subsystem.
 */
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

	/*
	  Creates a new ExampleCommand.
	 
	  @param subsystem The subsystem used by this command.
	*/
	public driveWithAprilTag(driveTrain driveTrain) {
		// Use requires() here to declare subsystem dependencies.
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	// Called when the command is initially scheduled.
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

	// Called every time the scheduler runs while the command is scheduled.
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

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
