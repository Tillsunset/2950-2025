package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends SubsystemBase {

	/*****************Position estimate variables**********************/
	public static double motorPowerToVelocity = 5880. * 3.1415 * 6./(39.37 * 60. * 8.45);
	public static double motorRPMToVelocity = 3.1415 * 6./(39.37 * 60. * 8.45);
	public double posX, posY;
	public double linearVel, prevLinearVel;

	private double prevLeftWheelVel = 0.0, prevRightWheelVel = 0.0;  // Previous wheel velocities for trapezoidal integration
	private double filteredLeftVel = 0.0, filteredRightVel = 0.0;  // Low-pass filtered velocities

	private static final double ALPHA = 0.8;  // Low-pass filter coefficient

	public double filteredyaw; // Smoothed gyro heading
	private static final double BETA = 0.8;  // Low-pass filter coefficient

	ADIS16470_IMU IMU = new ADIS16470_IMU();
	/*****************Position estimate variables**********************/

	// public SparkMax right = new SparkMax(2, MotorType.kBrushed);
	// public SparkMax left = new SparkMax(4, MotorType.kBrushed);
	// public DifferentialDrive driveBase = new DifferentialDrive(left, right);

	private SparkMax driveFR = new SparkMax(16, MotorType.kBrushless);
	private SparkMax driveBR = new SparkMax(20, MotorType.kBrushless);

	private SparkMax driveFL = new SparkMax(17, MotorType.kBrushless);
	private SparkMax driveBL = new SparkMax(3, MotorType.kBrushless);

	public DifferentialDrive driveBase = new DifferentialDrive(driveFL, driveFR);

	private RelativeEncoder right = driveFR.getEncoder();
	private RelativeEncoder left = driveFL.getEncoder();


	public driveTrain() {
		SparkMaxConfig globalConfig = new SparkMaxConfig();
		SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
		SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
		SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

		globalConfig
			.smartCurrentLimit(50)
			.inverted(true)
			.idleMode(IdleMode.kCoast);

		leftFollowerConfig
			.apply(globalConfig)
			.follow(driveFL);

		rightLeaderConfig
			.apply(globalConfig)
			.inverted(false);

		rightFollowerConfig
			.apply(rightLeaderConfig)
			.follow(driveFR);

		driveFR.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBR.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		driveFL.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBL.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		// left.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		// right.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		
		IMU.calibrate();
		resetPos();
	}

	@Override
	public void periodic() {
		updatePos();
		// printPosVelHead();
	}

	private void printPosVelHead() {
		System.out.print("X position: ");
		System.out.printf("%.2f\n", posX);

		System.out.print("Y position: ");
		System.out.printf("%.2f\n", posY);

		// System.out.print("linear veloctiy: ");
		// System.out.printf("%.2f\n", linearVel);

		System.out.print("heading: ");
		System.out.printf("%.2f\n", filteredyaw);
	}
	
	private void updatePos() {

		double dt = 0.02;

		filteredyaw = BETA * filteredyaw + (1 - BETA) * Math.toRadians(IMU.getAngle());

		filteredLeftVel = ALPHA * filteredLeftVel + (1 - ALPHA) * left.getVelocity() * motorRPMToVelocity;
        filteredRightVel = ALPHA * filteredRightVel + (1 - ALPHA) * right.getVelocity() * motorRPMToVelocity;

        // Compute linear and angular velocity using trapezoidal integration
        double linearVel = 0.5 * ((prevLeftWheelVel + filteredLeftVel) + (prevRightWheelVel + filteredRightVel)) / 2.0;
		
		posX += 0.5 * (linearVel + prevLinearVel) * Math.cos(filteredyaw) * dt;
        posY += 0.5 * (linearVel + prevLinearVel) * Math.sin(filteredyaw) * dt;

		// Store previous wheel velocities for next step
		prevLeftWheelVel = filteredLeftVel;
		prevRightWheelVel = filteredRightVel;
		prevLeftWheelVel = linearVel;
	}

	public void resetPos() {
		posX = 0.0;
        posY = 0.0;
		filteredyaw = 0.0;
		linearVel = 0.0;
		IMU.reset();
	}
}
