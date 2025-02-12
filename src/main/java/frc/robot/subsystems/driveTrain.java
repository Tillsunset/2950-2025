package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends SubsystemBase {
	private SparkMax driveFR = new SparkMax(2, MotorType.kBrushless);
	private SparkMax driveBR = new SparkMax(1, MotorType.kBrushless);
	private SparkMax driveFL = new SparkMax(3, MotorType.kBrushless);
	private SparkMax driveBL = new SparkMax(4, MotorType.kBrushless);

	public DifferentialDrive driveBase = new DifferentialDrive(driveFR, driveFL);

	public driveTrain() {
		SparkMaxConfig globalConfig = new SparkMaxConfig();
		SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
		SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
		SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

		globalConfig
			.smartCurrentLimit(20)
			.inverted(true)
			.idleMode(IdleMode.kBrake);

		leftFollowerConfig
			.apply(globalConfig)
			.follow(driveFL);

		rightLeaderConfig
			.apply(globalConfig)
			.inverted(false);

		rightFollowerConfig
			.apply(globalConfig)
			.follow(driveFR);

		driveFR.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBR.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveFL.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBL.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
	}
}
