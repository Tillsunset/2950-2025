package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilTest extends Command {
	private final driveTrain m_empty;

	public AprilTest(driveTrain empty) {
		m_empty = empty;
		addRequirements(empty);
		LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11});
	}

	@Override
	public void initialize() {
		m_empty.driveBase.stopMotor();
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
