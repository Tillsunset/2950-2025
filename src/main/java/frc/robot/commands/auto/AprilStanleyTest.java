package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain;
import frc.robot.Pose;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilStanleyTest extends Command {
	private final driveTrain m_driveTrain;

	/*******************Stanley Control variables**********************/
	private double k = 0.5; // how strongly correct cross error
    private double maxSteer = Math.toRadians(45);
    private double steerKp = 1; // tune steering strength

    double motorPower = 0.25;
	double zRotation = 0;
	double xSpeed = 0;
	/*******************Stanley Control variables**********************/

	/*******************Interpolation variables**********************/
	private static int interpolationPoints = 5;

	Pose current = new Pose(0, 0, 0);
    Pose goal = new Pose(0, 0, 0);

	List<Pose> waypoints = new ArrayList<>();
	/*******************Interpolation variables**********************/

	public AprilStanleyTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		m_driveTrain.resetPos();
	}

	@Override
	public void execute() {
		if (LimelightHelpers.getTV("")) {
			m_driveTrain.resetPos();
			current.x = 0;
			current.y = 0;
			current.yaw = 0;

			Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("");
            goal.x = pose.getZ();
            goal.y = pose.getX();
			goal.yaw = -pose.getRotation().getY();

			// waypoints.clear();
			waypoints = interpolateAlignedPoints(current, goal, interpolationPoints);
		}
		else {
			current.x = m_driveTrain.posX;
			current.y = m_driveTrain.posY;
			current.yaw = m_driveTrain.filteredyaw;
		}

		if (!waypoints.isEmpty()) {
			double steer = computeControl(current, waypoints);
			computeMotorPower(motorPower, steer);
		}
	}

	@Override
	public void end(boolean interrupted) {
        m_driveTrain.driveBase.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return distance(current.x - goal.x, current.y - goal.y) < 0.2;
	}

	private void printGoalMotor() {
		System.out.print("xSpeed: ");
		System.out.printf("%.2f\n", xSpeed);

		System.out.print("zRotation: ");
		System.out.printf("%.2f\n", zRotation);

		System.out.print("Goal: ");
		System.out.printf("%.2f %.2f %.2f\n", goal.x, goal.y, goal.yaw);
	}

	private double distance(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	private double computeControl(Pose current, List<Pose> waypoints) {
        double minDist = Double.MAX_VALUE;
		int nearestIdx = 0;
        
        for (int i = 0; i < waypoints.size(); i++) {
            double dist = distance(waypoints.get(i).x - current.x, waypoints.get(i).y - current.y);
            if (dist < minDist) {
                minDist = dist;
                nearestIdx = i;
            }
        }
        
        double targetX = waypoints.get(nearestIdx).x;
        double targetY = waypoints.get(nearestIdx).y;
        
        double pathYaw = Math.atan2(
            waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1)).y - targetY,
            waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1)).x - targetX
        );
        
        double crossTrackError = Math.sin(pathYaw) * (current.x - targetX) - Math.cos(pathYaw) * (current.y - targetY);

        double headingError = pathYaw - current.yaw;
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        
        double stanley_steer = headingError + Math.atan2(k * crossTrackError, (motorPower * driveTrain.motorToVelocity));
        return clamp(maxSteer, stanley_steer, -maxSteer);
    }

	private double clamp(double upper, double value, double lower) {
		return Math.max(lower, Math.min(upper, value));
	}

	private void computeMotorPower(double motorPower, double steer) {
		zRotation = steerKp * steer;
		xSpeed = motorPower;

		m_driveTrain.driveBase.arcadeDrive(xSpeed, zRotation, false);
    }

	private List<Pose> interpolateAlignedPoints(Pose current, Pose goal, int numPoints) {
        List<Pose> points = new ArrayList<>();
        
        double directionX = Math.cos(goal.yaw);
        double directionY = Math.sin(goal.yaw);
        
        // Project current position onto the goal's tangent line
        double displacementX = current.x - goal.x;
        double displacementY = current.y - goal.y;
        double projectionLength = displacementX * directionX + displacementY * directionY;
        double projectedStartX = goal.x + projectionLength * directionX;
        double projectedStartY = goal.y + projectionLength * directionY;
        
        // Generate evenly spaced points along the tangent line
        double totalDistance = Math.sqrt(Math.pow(goal.x - projectedStartX, 2) + Math.pow(goal.y - projectedStartY, 2));
        double spacing = totalDistance / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++) {
            double x = projectedStartX + i * spacing * directionX;
            double y = projectedStartY + i * spacing * directionY;
            points.add(new Pose(x, y, goal.yaw));
        }
        
        return points;
    }
}
