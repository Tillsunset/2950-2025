package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.empty;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

class Pose {
    double x, y, yaw;

    public Pose(double x, double y, double yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }
}

public class accelAndGyroTest extends Command {
	private final driveTrain m_driveTrain;

	/*******************Stanley Control variables**********************/
	private double k = 0.5;
    private double kSoft = 0.001;
    private double maxSteer = Math.toRadians(20);
    private double steerKp = 1;
	private double powerKp = 2;
	int nearestIdx = 0;

	List<Pose> waypoints;
	/*******************Stanley Control variables**********************/

	public accelAndGyroTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		// m_driveTrain.resetPos();

        Pose start = new Pose(0, 0, 0);
        Pose goal = new Pose(5, 5, Math.PI / 2);
        double maxVelocity = 1.0;
        double timeStep = 0.1;
        double maxAngularRate = Math.toRadians(30);
		waypoints = interpolateTowardsEachOther(start, goal, maxVelocity, timeStep, maxAngularRate);

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

	private double computeControl(double x, double y, double yaw, double v, List<Pose> waypoints) {
        double minDist = Double.MAX_VALUE;
        // int nearestIdx = 0;
        
        for (int i = 0; i < waypoints.size(); i++) {
            double dx = waypoints.get(i).x - x;
            double dy = waypoints.get(i).y - y;
            double dist = Math.sqrt(dx * dx + dy * dy);
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

	public static List<Pose> interpolateTowardsEachOther(Pose start, Pose goal, double maxVelocity, double timeStep, double maxAngularRate) {
        List<Pose> pathStart = new ArrayList<>();
        List<Pose> pathGoal = new ArrayList<>();
        
        pathStart.add(start);
        pathGoal.add(goal);
        
        Pose currentStart = start;
        Pose currentGoal = goal;
        
        while (distance(currentStart, currentGoal) > maxVelocity * timeStep) {
            // Compute direction vectors
            double startDirX = Math.cos(currentStart.yaw);
            double startDirY = Math.sin(currentStart.yaw);
            double goalDirX = Math.cos(currentGoal.yaw);
            double goalDirY = Math.sin(currentGoal.yaw);
            
            double stepSize = maxVelocity * timeStep;
            
            // Compute new positions
            Pose newStart = new Pose(
                currentStart.x + startDirX * stepSize,
                currentStart.y + startDirY * stepSize,
                currentStart.yaw
            );
            Pose newGoal = new Pose(
                currentGoal.x - goalDirX * stepSize,
                currentGoal.y - goalDirY * stepSize,
                currentGoal.yaw
            );
            
            // Compute desired yaw
            double desiredYaw = Math.atan2(newGoal.y - newStart.y, newGoal.x - newStart.x);
            
            // Limit angular velocity change
            double yawChange = maxAngularRate * timeStep;
            newStart.yaw = clamp(desiredYaw, currentStart.yaw - yawChange, currentStart.yaw + yawChange);
            newGoal.yaw = clamp(desiredYaw, currentGoal.yaw - yawChange, currentGoal.yaw + yawChange);
            
            pathStart.add(newStart);
            pathGoal.add(newGoal);
            
            currentStart = newStart;
            currentGoal = newGoal;
        }
        
        // Reverse goal path and merge
        List<Pose> fullPath = new ArrayList<>(pathStart);
        for (int i = pathGoal.size() - 2; i >= 0; i--) {
            fullPath.add(pathGoal.get(i));
        }
        
        // Remove redundant points
        return removeRedundantPoints(fullPath);
    }

    private static double distance(Pose a, Pose b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static List<Pose> removeRedundantPoints(List<Pose> path) {
        List<Pose> filteredPath = new ArrayList<>();
        filteredPath.add(path.get(0));
        
        for (int i = 1; i < path.size() - 1; i++) {
            Pose prev = filteredPath.get(filteredPath.size() - 1);
            Pose curr = path.get(i);
            Pose next = path.get(i + 1);
            
            double dir1X = (curr.x - prev.x) / distance(curr, prev);
            double dir1Y = (curr.y - prev.y) / distance(curr, prev);
            double dir2X = (next.x - curr.x) / distance(next, curr);
            double dir2Y = (next.y - curr.y) / distance(next, curr);
            
            double dotProduct = dir1X * dir2X + dir1Y * dir2Y;
            if (dotProduct < 0.999 || Math.abs(curr.yaw - prev.yaw) > 0.001) {
                filteredPath.add(curr);
            }
        }
        
        filteredPath.add(path.get(path.size() - 1));
        return filteredPath;
    }
}
