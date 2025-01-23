/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends SubsystemBase {

	private SparkMax driveFR = new SparkMax(1, MotorType.kBrushless);
	private SparkMax driveBR = new SparkMax(2, MotorType.kBrushless);
	private SparkMax driveFL = new SparkMax(3, MotorType.kBrushless);
	private SparkMax driveBL = new SparkMax(4, MotorType.kBrushless);

	// private WPI_TalonSRX driveFR = talonSRXConstructor(1);
	// private WPI_TalonSRX driveBR = talonSRXConstructor(2);
	// private WPI_TalonSRX driveFL = talonSRXConstructor(3);
	// private WPI_TalonSRX driveBL = talonSRXConstructor(4);

	public DifferentialDrive driveBase = new DifferentialDrive(driveFR, driveFL);

	public driveTrain() {
		// driveFR.setInverted(InvertType.InvertMotorOutput);
		// driveBR.follow(driveFR);
		// driveBR.setInverted(InvertType.FollowMaster);
		
		// driveFL.setInverted(InvertType.None);
		// driveBL.follow(driveFL);
		// driveBL.setInverted(InvertType.FollowMaster);

		sparkMaxConfigureHelper(driveFR, null, true);
		sparkMaxConfigureHelper(driveBR, driveFR, false);
		sparkMaxConfigureHelper(driveFL, null, false);
		sparkMaxConfigureHelper(driveBL, driveFL, false);
	}

	@Override
	public void periodic() {
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

	// private WPI_TalonSRX talonSRXConstructor(int x) {
	// 	WPI_TalonSRX temp = new WPI_TalonSRX(x);

	// 	temp.configContinuousCurrentLimit(20);// used standard play
	// 	temp.configPeakCurrentLimit(40, 1000);// used for pushing, limit for stopping wheel spin
	// 	temp.enableCurrentLimit(true);
	// 	// temp.configOpenloopRamp(.25);// fine tune for best responsiveness
	// 	// temp.configClosedloopRamp(0);// used for driving by encoders
	// 	temp.setNeutralMode(NeutralMode.Brake);

	// 	return temp;
	// }
}
