package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.empty;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class accelAndGyroTest extends Command {
	private final driveTrain m_driveTrain;

	/*******************Stanley Control variables**********************/
	private double k = 0.5;
    private double kSoft = 0.001;
    private double maxSteer = Math.toRadians(20);
    private double steerKp = 1;
	private double powerKp = 2;
	int nearestIdx = 0;


	List<double[]> waypoints = Arrays.asList(
		new double[]{0, 0},
		new double[]{0.1, 1},
		new double[]{0, 2}

	);
	/*******************Stanley Control variables**********************/

	public accelAndGyroTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		// m_driveTrain.resetPos();
	}

	@Override
	public void execute() {

		double steer = computeControl(m_driveTrain.posX, m_driveTrain.posY, m_driveTrain.filteredyaw, m_driveTrain.filteredyaw, waypoints);
		computeMotorPower(0.5, steer);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return nearestIdx == (waypoints.size() - 1);
	}

	private double normalize(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	private double computeControl(double x, double y, double yaw, double v, List<double[]> waypoints) {
        double minDist = Double.MAX_VALUE;
        // int nearestIdx = 0;
        
        for (int i = 0; i < waypoints.size(); i++) {
            double dx = waypoints.get(i)[0] - x;
            double dy = waypoints.get(i)[1] - y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < minDist) {
                minDist = dist;
                nearestIdx = i;
            }
        }
        
        double targetX = waypoints.get(nearestIdx)[0];
        double targetY = waypoints.get(nearestIdx)[1];
        
        double pathYaw = Math.atan2(
            waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1))[1] - targetY,
            waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1))[0] - targetX
        );
        
        double crossTrackError = Math.sin(pathYaw) * (x - targetX) - Math.cos(pathYaw) * (y - targetY);
        double headingError = pathYaw - yaw;
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        
        double steer = headingError + Math.atan2(k * crossTrackError, (kSoft + v));
        return Math.max(-maxSteer, Math.min(maxSteer, steer));
    }

	public void computeMotorPower(double v, double steer) {
		double zRotation = steer * steerKp;
		double motorPower = powerKp* v/m_driveTrain.motorToVelocity;

		System.out.print("steer");
		System.out.println(zRotation);
		System.out.print("motorPower");
		System.out.println(motorPower);

		m_driveTrain.driveBase.arcadeDrive(motorPower, zRotation, false);
    }
}
