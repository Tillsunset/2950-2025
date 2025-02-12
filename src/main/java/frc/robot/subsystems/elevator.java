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
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

	private SparkMax front = new SparkMax(1, MotorType.kBrushless);
	private SparkMax back = new SparkMax(2, MotorType.kBrushless);
	
	private SparkClosedLoopController closedLoopController = front.getClosedLoopController();

	private double inchToPos = 9 * 42 * 1.76 * Math.PI;
	private double feedForward = 0.1;

	public elevator() {
		SparkMaxConfig leaderConfig = new SparkMaxConfig();
		SparkMaxConfig followerConfig = new SparkMaxConfig();

		leaderConfig.smartCurrentLimit(20)
			.idleMode(IdleMode.kBrake);

		leaderConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.p(0.001)
			.i(0)
			.d(0)
			.outputRange(-1, 1);

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
		closedLoopController.setReference(target * inchToPos, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kPercentOut);
	}
}
