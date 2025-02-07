/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
		/*
		* Create new SPARK MAX configuration objects. These will store the
		* configuration parameters for the SPARK MAXes that we will set below.
		*/
		SparkMaxConfig globalConfig = new SparkMaxConfig();
		SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
		SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
		SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

		/*
		* Set parameters that will apply to all SPARKs. We will also use this as
		* the left leader config.
		*/
		globalConfig
			.smartCurrentLimit(20)
			.inverted(true)
			.idleMode(IdleMode.kBrake);

		// Apply the global config and set the leader SPARK for follower mode
		leftFollowerConfig
			.apply(globalConfig)
			.follow(driveFL);

		// Apply the global config and invert since it is on the opposite side
		rightLeaderConfig
		.apply(globalConfig)
		.inverted(false);

		// Apply the global config and set the leader SPARK for follower mode
		rightFollowerConfig
			.apply(globalConfig)
			.follow(driveFR);

		/*
		* Apply the configuration to the SPARKs.
		*
		* kResetSafeParameters is used to get the SPARK MAX to a known state. This
		* is useful in case the SPARK MAX is replaced.
		*
		* kPersistParameters is used to ensure the configuration is not lost when
		* the SPARK MAX loses power. This is useful for power cycles that may occur
		* mid-operation.
		*/
		driveFR.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBR.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveFL.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBL.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
	}
}
