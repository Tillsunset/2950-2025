/*----------------------------------------------------------------------------/
/ Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        /
/ Open Source Software - may be modified and shared by FRC teams. The code   /
/ must be accompanied by the FIRST BSD license file in the root directory of /
/ the project.                                                               /
/----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/*
  An example command that uses an example subsystem.
 */
public class driveWithAprilTag extends Command {
	private final driveTrain m_driveTrain;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	double forwardGoalDistance = 3;
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

	enum t2dEnum
	{
		targetValid(0),
		targetCount(1),
		targetLatency(2),
		captureLatency(3),
		tx(4),
		ty(5),
		txnc(6),
		tync(7),
		ta(8),
		tid(9),
		targetClassIndexDetector(10),
		targetClassIndexClassifier(11),
		targetLongSidePixels(12),
		targetShortSidePixels(13),
		targetHorizontalExtentPixels(14),
		targetVerticalExtentPixels(15),
		targetSkewDegrees(16);

		private final int index;

		private t2dEnum(int index) 
		{ 
			this.index = index; 
		} 
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double[] t2d = table.getEntry("t2d").getDoubleArray(new double[0]);
		if (t2d.length == 17 && t2d[t2dEnum.targetValid.index] > 0) {
			double targetAngleVerticalDegrees = t2d[t2dEnum.ty.index];
			
			double angleToGoalRadians = (limelightMountAngleDegrees + targetAngleVerticalDegrees) * (3.14159 / 180.0);
		
			double distanceToAprilTagMeters = (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);

			double targetSkewDegrees = t2d[t2dEnum.targetSkewDegrees.index];
		}
		else {
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
