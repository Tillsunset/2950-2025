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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

	private SparkMax front = new SparkMax(1, MotorType.kBrushless);
	private SparkMax back = new SparkMax(2, MotorType.kBrushless);
	
	private SparkClosedLoopController closedLoopController = front.getClosedLoopController();

	public elevator() {
		SparkMaxConfig leaderConfig = new SparkMaxConfig();
		SparkMaxConfig followerConfig = new SparkMaxConfig();

		leaderConfig.smartCurrentLimit(20)
			.idleMode(IdleMode.kBrake);

		/*
		* Configure the closed loop controller. We want to make sure we set the
		* feedback sensor as the primary encoder.
		*/
		leaderConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			// Set PID values for position control. We don't need to pass a closed loop
			// slot, as it will default to slot 0.
			.p(0.001)
			.i(0)
			.d(0)
			.outputRange(-1, 1);

		/*
		* Apply the configuration to the SPARK MAX.
		*
		* kResetSafeParameters is used to get the SPARK MAX to a known state. This
		* is useful in case the SPARK MAX is replaced.
		*
		* kPersistParameters is used to ensure the configuration is not lost when
		* the SPARK MAX loses power. This is useful for power cycles that may occur
		* mid-operation.
		*/

		followerConfig
			.apply(leaderConfig)
			.follow(front)
			.inverted(true);

		front.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		back.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {}

	public void updateTargetPosition(double target) {
		closedLoopController.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
	}
}
