package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class alignUsingAprilTag extends Command {
	private final driveTrain m_driveTrain;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private Timer m_timer;  

	double forwardGoal = 0.5;
	double forwardDistance = 0;
	double forwardError = 0;
	double kPDistance = 0.01;

	double sideGoal = 0.5;
	double sideDistance = 0;
	double sideOError = 0;
	double kPSide = 0.01;

	double tv = 0;

	public alignUsingAprilTag(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	@Override
	public void initialize() {
		m_timer.reset();
        m_timer.start();
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
		tv += table.getEntry("tv").getDouble(0.0);
		tv /= 2;

		if (t2d.length == 6 && tv > 0.8) {
			sideDistance = t2d[t2dEnum.tx.val];
			sideOError = sideDistance - sideGoal;

			forwardDistance = t2d[t2dEnum.ty.val];
			forwardError = forwardDistance - forwardGoal;

			m_driveTrain.driveBase.arcadeDrive(forwardError * kPDistance, sideOError * kPSide, false);
		}
		else {
			m_driveTrain.driveBase.feed();
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_driveTrain.driveBase.stopMotor();
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return forwardError < 0.1 || m_timer.get() > 5;
	}
}
