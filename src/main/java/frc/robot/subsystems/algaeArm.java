package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeArm extends SubsystemBase {
	private SparkMax algeaArm = new SparkMax(14, MotorType.kBrushless);
	
	private SparkClosedLoopController closedLoopController = algeaArm.getClosedLoopController();

	private RelativeEncoder encoder = algeaArm.getEncoder();

	private double feedForward = 0.1;

	public algaeArm(){
		SparkMaxConfig algeaArmConfig = new SparkMaxConfig();

		algeaArmConfig
			.smartCurrentLimit(9)
			.idleMode(IdleMode.kBrake);

		algeaArmConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.p(0.015)
			.d(0)
			.outputRange(-1, 1);

		algeaArm.configure(algeaArmConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		encoder.setPosition(0);
	}

	@Override
	public void periodic() {}

	public void updateTargetAngle(double target) {
		closedLoopController.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kPercentOut);
	}
}