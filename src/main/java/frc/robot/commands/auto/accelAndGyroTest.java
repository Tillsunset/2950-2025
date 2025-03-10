package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.empty;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;

public class accelAndGyroTest extends Command {
	private final empty m_driveTrain;

	/*****************Position estimate variables**********************/
	private double posX, posY;  // Position (x, y)
    private double velX, velY;  // Velocity (x, y)
	private double prevAccelX, prevAccelY;
    private double filteredAccelX, filteredAccelY;  // Smoothed acceleration values
    private double filteredyaw; // Smoothed gyro heading
    private static final double ALPHA = 0.8;  // Low-pass filter coefficient
    private static final double BETA = 0.8;  // Low-pass filter coefficient
	private static final double g = 9.81; // G
	private static final double drag = .98; // drag to reduce steady state error accumulation
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

	ADIS16470_IMU IMU = new ADIS16470_IMU();

	public accelAndGyroTest(empty driveTrain) {
		IMU.calibrate();
		m_driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		resetPos();
	}

	@Override
	public void execute() {

		updatePos();
		System.out.print("X position: ");
		System.out.printf("%.2f\n", posX);
		System.out.print("X veloctiy: ");
		System.out.printf("%.2f\n", velX);
		System.out.print("X accel: ");
		System.out.printf("%.2f\n", filteredAccelX);

		System.out.print("Y position: ");
		System.out.printf("%.2f\n", posY);
		System.out.print("Y veloctiy: ");
		System.out.printf("%.2f\n", velY);
		System.out.print("Y accel: ");
		System.out.printf("%.2f\n", filteredAccelY);

		System.out.print("heading: ");
		System.out.printf("%.2f\n", filteredyaw);

		// double steer = computeControl(posX, posY, filteredyaw, normalize(velX, velY), waypoints);
		// computeMotorPower(0.5, steer);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
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
		// m_driveTrain.driveBase.arcadeDrive(v, zRotation, false)
    }

	private void updatePos() {

		double dt = 0.02;

		filteredyaw = BETA * filteredyaw + (1 - BETA) * Math.toRadians(IMU.getAngle());

		double accelX = IMU.getAccelX();
		double accelY = IMU.getAccelY();

		double gX = -.15;
		double gY = .18;
		
		accelX += gX;
		accelY += gY;

        filteredAccelX = ALPHA * filteredAccelX + (1 - ALPHA) * accelX;
        filteredAccelY = ALPHA * filteredAccelY + (1 - ALPHA) * accelY;
		
        // Rotate filtered acceleration into world frame
        double worldAccelX = filteredAccelX * Math.cos(filteredyaw) - filteredAccelY * Math.sin(filteredyaw);
        double worldAccelY = filteredAccelX * Math.sin(filteredyaw) + filteredAccelY * Math.cos(filteredyaw);

        // Trapezoidal Integrate acceleration to update velocity
		velX += 0.5 * (prevAccelX + worldAccelX) * dt;
		velY += 0.5 * (prevAccelY + worldAccelY) * dt;

		velX *= drag;
		velY *= drag;

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
		filteredyaw = 0.0;
	}
}
