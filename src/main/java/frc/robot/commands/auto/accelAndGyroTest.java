package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.Command;

public class accelAndGyroTest extends Command {
	private final driveTrain m_driveTrain;

	/*****************Position estimate variables**********************/
	private double posX, posY;  // Position (x, y)
    private double velX, velY;  // Velocity (x, y)
	private double prevAccelX, prevAccelY;
    private double filteredAccelX, filteredAccelY;  // Smoothed acceleration values
	private double gyroHeading;
    private double prevGyroRate;
    private double filteredGyroHeading; // Smoothed gyro heading
    private static final double ALPHA = 0.8;  // Low-pass filter coefficient
    private static final double BETA = 0.8;  // Low-pass filter coefficient
	/*****************Position estimate variables**********************/

	/*******************Stanley Control variables**********************/
	private double k = 0.5;
    private double kSoft = 0.001;
    private double maxSteer = Math.toRadians(30);
    private double steerKp = 0.5;

	List<double[]> waypoints = Arrays.asList(
		new double[]{1, 1},
		new double[]{1.1, 1.1}
	);
	/*******************Stanley Control variables**********************/

	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	BuiltInAccelerometer accel = new BuiltInAccelerometer();


	double tv = 0;

	public accelAndGyroTest(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
		LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11});
	}

	@Override
	public void initialize() {
		resetPos();
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
			
			// pose.getRotation().
		}
		else {
			System.out.println("not found");
		}

		updatePos();
		System.out.print("X position: ");
		System.out.println(posX);
		System.out.print("Y position: ");
		System.out.println(posY);
		// double steer = computeControl(posX, posY, filteredGyroHeading, normalize(velX, velY), waypoints);
		// computeMotorPower(0.5, steer);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveTrain.driveBase.stopMotor();
	}

	@Override
	public boolean isFinished() {
		int lastIdx = waypoints.size() - 1;
		return normalize(posX - waypoints.get(lastIdx)[0], posY - waypoints.get(lastIdx)[1]) < 0.5;
	}

	private double normalize(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	private double computeControl(double x, double y, double yaw, double v, List<double[]> waypoints) {
        double minDist = Double.MAX_VALUE;
        int nearestIdx = 0;
        
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

	public void computeMotorPower(double steer, double v) {
		double zRotation = steer * steerKp;
		m_driveTrain.driveBase.arcadeDrive(v, zRotation, false);
    }

	private void updatePos() {

		double dt = 0.02;

		// double filteredGyroRate = BETA * prevGyroRate + (1 - BETA) * gyro.getRate();
		// gyroHeading += 0.5 * (prevGyroRate + filteredGyroRate) * dt;
		// prevGyroRate = filteredGyroRate;
		gyroHeading = Math.toRadians(gyro.getAngle());

		filteredGyroHeading = BETA * filteredGyroHeading + (1 - BETA) * gyroHeading;
	
		double accelX = accel.getX();
		double accelY = accel.getY();

        filteredAccelX = ALPHA * filteredAccelX + (1 - ALPHA) * accelX;
        filteredAccelY = ALPHA * filteredAccelY + (1 - ALPHA) * accelY;
		

        // Rotate filtered acceleration into world frame
        double worldAccelX = filteredAccelX * Math.cos(filteredGyroHeading) - filteredAccelY * Math.sin(filteredGyroHeading);
        double worldAccelY = filteredAccelX * Math.sin(filteredGyroHeading) + filteredAccelY * Math.cos(filteredGyroHeading);

        // Trapezoidal Integrate acceleration to update velocity
		velX += 0.5 * (prevAccelX + worldAccelX) * dt;
		velY += 0.5 * (prevAccelY + worldAccelY) * dt;

        // Integrate velocity to update position
        posX += velX * dt;
        posY += velY * dt;

		// Update previous acceleration values
		prevAccelX = worldAccelX;
		prevAccelY = worldAccelY;
	}

	private void resetPos() {
		posX = 0.0;
        posY = 0.0;
        velX = 0.0;
        velY = 0.0;
		prevAccelX = 0.0;
		prevAccelY = 0.0;
		filteredAccelX = 0.0;
		filteredAccelY = 0.0;
		prevGyroRate = 0.0;
		filteredGyroHeading = 0.0;
	}
}
