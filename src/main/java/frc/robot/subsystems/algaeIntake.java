package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

public class algaeIntake extends SubsystemBase {

	// private WPI_VictorSPX algeaIntake = new WPI_VictorSPX(1);
	public SparkMax algeaIntake = new SparkMax(19, MotorType.kBrushed);

	public algaeIntake() {
		SparkMaxConfig algeaIntakeConfig = new SparkMaxConfig();

		algeaIntakeConfig
			.smartCurrentLimit(20)
			.idleMode(IdleMode.kBrake);

		algeaIntake.configure(algeaIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}
	
	public void setOutput(double output) {
		algeaIntake.set(output);
	}
}
