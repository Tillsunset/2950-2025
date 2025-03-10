package frc.robot.commands.auto;

import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.empty;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;

public class accelAndGyroTest extends Command {
	private final driveTrain m_driveTrain;

	/*****************Position estimate variables**********************/

	private double motorToVelocity = 8.45 * Math.PI * 6 / 39.37;
	private double wheelBase = .65;  // Distance between left and right wheels
	private double posX, posY;
	private double linearVel, prevLinearVel;

    private double prevLeftWheelVel = 0.0, prevRightWheelVel = 0.0;  // Previous wheel velocities for trapezoidal integration
    private double filteredLeftVel = 0.0, filteredRightVel = 0.0;  // Low-pass filtered velocities

    private static final double ALPHA = 0.8;  // Low-pass filter coefficient

    private double filteredyaw; // Smoothed gyro heading
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

	ADIS16470_IMU IMU = new ADIS16470_IMU();

	public accelAndGyroTest(driveTrain driveTrain) {
		IMU.calibrate();
		m_driveTrain = driveTrain;
		// addRequirements(driveTrain);
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

		System.out.print("Y position: ");
		System.out.printf("%.2f\n", posY);

		System.out.print("linear veloctiy: ");
		System.out.printf("%.2f\n", linearVel);


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

		filteredLeftVel = ALPHA * filteredLeftVel + (1 - ALPHA) * m_driveTrain.left.get() * motorToVelocity;
        filteredRightVel = ALPHA * filteredRightVel + (1 - ALPHA) * m_driveTrain.right.get() * motorToVelocity;

        // Compute linear and angular velocity using trapezoidal integration
        double linearVel = 0.5 * ((prevLeftWheelVel + filteredLeftVel) + (prevRightWheelVel + filteredRightVel)) / 2.0;
		
		posX += 0.5 * (linearVel + prevLinearVel) * Math.cos(filteredyaw) * dt;
        posY += 0.5 * (linearVel + prevLinearVel) * Math.sin(filteredyaw) * dt;

		// Store previous wheel velocities for next step
		prevLeftWheelVel = filteredLeftVel;
		prevRightWheelVel = filteredRightVel;
		prevLeftWheelVel = linearVel;
	}

	private void resetPos() {
		posX = 0.0;
        posY = 0.0;
		filteredyaw = 0.0;
		linearVel = 0.0;
	}
}
