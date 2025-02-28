package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class winch extends SubsystemBase {

	public SparkMax winch = new SparkMax(19, MotorType.kBrushless);
	
	private SparkClosedLoopController closedLoopController = winch.getClosedLoopController();

	private double inchToPos = 90 * 42 * 0.5 * Math.PI;
	private double feedForward = 0.1;

	public winch(){
		SparkMaxConfig winchConfig = new SparkMaxConfig();

		winchConfig
			.smartCurrentLimit(40)
			.idleMode(IdleMode.kBrake);

		// winchConfig.closedLoop
		// 	.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// 	.p(0.001)
		// 	.i(0)
		// 	.d(0)
		// 	.outputRange(-1, 1);

		winch.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {}

	public void updateTargetPosition(double target) {
		closedLoopController.setReference(target * inchToPos, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kPercentOut);
	}
}