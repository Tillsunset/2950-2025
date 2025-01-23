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
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

	private SparkMax front = new SparkMax(1, MotorType.kBrushless);
	private SparkMax back = new SparkMax(2, MotorType.kBrushless);
	private SparkAbsoluteEncoder encoder = front.getAbsoluteEncoder();

	public double setPoint = 0;
	private double encoderStart;
	private double kF = 0;
	private double kP = 0;
	private double kD = 0;

	public elevator() {
		sparkMaxConfigureHelper(front, null, true);
		sparkMaxConfigureHelper(back, front, false);
		encoderStart = encoder.getPosition();
	}

	@Override
	public void periodic() {
		double posError = setPoint - (encoder.getPosition() - encoderStart);

		front.set(kF + 
					posError * kP + 
					encoder.getVelocity() * kD);
	}

	private void sparkMaxConfigureHelper(SparkMax x, SparkMax primary, boolean invert) {
		SparkMaxConfig temp = new SparkMaxConfig();
		if (primary != null) {
			temp.follow(primary, invert);
		}
		else {
			temp.inverted(invert);
		}
		temp.idleMode(IdleMode.kBrake);
		temp.smartCurrentLimit(20);

		x.configure(temp, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}
}
