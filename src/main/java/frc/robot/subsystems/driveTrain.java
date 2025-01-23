/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends SubsystemBase {
	public WPI_TalonSRX driveFL = talonSRXConstructor(3);
	public WPI_TalonSRX steerFL = talonSRXConstructor(7);

	public WPI_TalonSRX driveBL = talonSRXConstructor(4);
	public WPI_TalonSRX steerBL = talonSRXConstructor(8);
	
	public WPI_TalonSRX driveBR = talonSRXConstructor(2);
	public WPI_TalonSRX steerBR = talonSRXConstructor(6);

	public WPI_TalonSRX driveFR = talonSRXConstructor(1);
	public WPI_TalonSRX steerFR = talonSRXConstructor(5);

	public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public driveTrain() {
		driveFL.setInverted(InvertType.None);
		steerFL.setInverted(InvertType.None);

		driveBL.setInverted(InvertType.None);
		steerBL.setInverted(InvertType.None);

		driveBR.setInverted(InvertType.None);
		steerBR.setInverted(InvertType.None);

		driveFR.setInverted(InvertType.None);
		steerFR.setInverted(InvertType.None);

		steerFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		steerBL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		steerBR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		steerFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
	}

	@Override
	public void periodic() {
	}

	private WPI_TalonSRX talonSRXConstructor(int x) {
		WPI_TalonSRX temp = new WPI_TalonSRX(x);

		temp.enableCurrentLimit(true);
		temp.configPeakCurrentLimit(20, 1000);// used for pushing, limit for stopping wheel spin
		temp.configContinuousCurrentLimit(20);// used standard play
		// temp.configOpenloopRamp(.25);// fine tune for best responsiveness
		// temp.configClosedloopRamp(0);// used for driving by encoders
		temp.setNeutralMode(NeutralMode.Brake);

		return temp;
	}
}
