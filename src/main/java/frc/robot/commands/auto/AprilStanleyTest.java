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
	private double k = 0.5;
    private double kSoft = 0.001;
    private double maxSteer = Math.toRadians(30);
    private double steerKp = .5;
	private double powerKp = 0.3;
	int nearestIdx = 0;

	List<Pose> waypoints = new ArrayList<>();

    Pose start = new Pose(0, 0, 0);
    Pose goal = new Pose(1, 0.01, 0);
    double motorPower = 1;
    double timeStep = 0.03;
	/*******************Stanley Control variables**********************/

	public AprilStanleyTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		m_driveTrain.resetPos();
        waypoints.clear();

        if (LimelightHelpers.getTV("")) {
			Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("");

            goal.x = pose.getZ();
            goal.y = pose.getX();
        }
    	waypoints = interpolateTowardsEachOther(start, goal, motorPower, timeStep, maxSteer);
        System.gc();
	}

	@Override
	public void execute() {
		double steer = computeControl(m_driveTrain.posX, m_driveTrain.posY, m_driveTrain.filteredyaw, motorPower, waypoints);
		computeMotorPower(1, steer);

        // System.out.println("goal positions:");
        // System.out.println(goal.x);
        // System.out.println(goal.y);
        System.out.println("waypoints Numebr:");
        System.out.println(waypoints.size());
	}

	@Override
	public void end(boolean interrupted) {
        m_driveTrain.driveBase.stopMotor();
        waypoints.clear();
        System.gc();
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
        
        for (int i = 0; i < waypoints.size(); i++) {
            double dist = normalize(waypoints.get(i).x - x, waypoints.get(i).y - y);
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
        
        double steer = headingError + Math.atan2(k * crossTrackError, (kSoft + v * driveTrain.motorToVelocity));
        return Math.max(-maxSteer, Math.min(maxSteer, steer));
    }

	public void computeMotorPower(double motorPower, double steer) {
		double zRotation = steerKp * steer;
		double xSpeed = powerKp * motorPower;

		// System.out.print("steer");
		// System.out.println(zRotation);
		// System.out.print("motorPower");
		// System.out.println(xSpeed);

		m_driveTrain.driveBase.arcadeDrive(xSpeed, zRotation, false);
    }

	public static List<Pose> interpolateTowardsEachOther(Pose start, Pose goal, double motorPower, double timeStep, double maxAngularRate) {
        List<Pose> pathStart = new ArrayList<>();
        List<Pose> pathGoal = new ArrayList<>();
        
        pathStart.add(start);
        pathGoal.add(goal);
        
        Pose currentStart = start;
        Pose currentGoal = goal;
        
        while (distance(currentStart, currentGoal) > motorPower * driveTrain.motorToVelocity * timeStep) {
            // Compute direction vectors
            double startDirX = Math.cos(currentStart.yaw);
            double startDirY = Math.sin(currentStart.yaw);
            double goalDirX = Math.cos(currentGoal.yaw);
            double goalDirY = Math.sin(currentGoal.yaw);
            
            double stepSize = motorPower * timeStep;
            
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
        for (int i = pathGoal.size() - 2; i >= 0; i--) {
            pathStart.add(pathGoal.get(i));
        }

        pathGoal.clear();
        
        // return fullPath;
        // Remove redundant points
        List<Pose> outPath = removeRedundantPoints(pathStart);

        pathStart.clear();
        return outPath;
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
            if (dotProduct < 0.999 || Math.abs(curr.yaw - prev.yaw) > Math.toRadians(.01)) {
                filteredPath.add(curr);
            }
        }
        
        filteredPath.add(path.get(path.size() - 1));
        path.clear();
    
        return filteredPath;
    }
}
