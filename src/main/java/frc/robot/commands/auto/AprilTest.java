package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilTest extends Command {
	// private final empty m_empty;
	private final driveTrain m_empty;

   
	double forwardGoal = .1;
	double forwardDistance;
	double forwardError;
	double kPDistance = 1;

	double sideGoal = 0.0;
	double sideDistance;
	double sideOError;
	double kPSide = 1.;

	double tv = 0;

	public AprilTest(driveTrain empty) {
		m_empty = empty;
		addRequirements(empty);
		LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11});
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (LimelightHelpers.getTV("")) {
			Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("");
			System.out.println("found AprilTag");
			System.out.print("distance: ");
			System.out.println(pose.getZ());
			System.out.print("offset: ");
			System.out.println(pose.getX());
			System.out.println(-pose.getRotation().getY());

		}
		else {
			System.out.println("not found");
			m_empty.driveBase.feed();
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_empty.driveBase.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
