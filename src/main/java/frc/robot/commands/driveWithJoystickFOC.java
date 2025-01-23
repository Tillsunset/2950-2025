/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.driveTrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * An example command that uses an example subsystem.
 */
public class driveWithJoystickFOC extends Command {
	private final driveTrain m_driveTrain;
	private DoubleSupplier leftAxisX;
	private DoubleSupplier leftAxisY;
	private DoubleSupplier rightAxisX;
	private DoubleSupplier rightAxisY;

	private double currentHeading = 0;
	private double maxTurnRate = Math.PI/10;
	private double motorHeadingOffset = Math.PI * 1/2;
	private double motorToCenterDistance = 1;
	private double deltaHeading = 0;

	private ArrayList<Double> deltaXList = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
	private ArrayList<Double> deltaYList = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
	private ArrayList<Double> magList = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
	private ArrayList<Double> dirList = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0, 0.0));

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public driveWithJoystickFOC(driveTrain driveTrain, XboxController x) {
		leftAxisX = x::getLeftX;
		leftAxisY = x::getLeftY;
		rightAxisX = x::getRightX;
		rightAxisY = x::getRightY;
		// Use requires() here to declare subsystem dependencies.
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		currentHeading = m_driveTrain.gyro.getAngle() * Math.PI / 180;

		double targetHeading = atan2(rightAxisY.getAsDouble(), rightAxisX.getAsDouble());
		double targetHeadingScaling = magnitude(rightAxisX.getAsDouble(), rightAxisY.getAsDouble());
		deltaHeading = getDeltaHeading(currentHeading, targetHeading, targetHeadingScaling);

		double deltaX = leftAxisX.getAsDouble();
		double deltaY = leftAxisY.getAsDouble();
		double mag = magnitude(leftAxisX.getAsDouble(), leftAxisY.getAsDouble());

		deltaXList.set(0, getDeltaTranslation(deltaX, 1));
		deltaXList.set(1, getDeltaTranslation(deltaX, 3));
		deltaXList.set(2, getDeltaTranslation(deltaX, 5));
		deltaXList.set(3, getDeltaTranslation(deltaX, 7));

		deltaYList.set(0, getDeltaTranslation(deltaY, 1));
		deltaYList.set(1, getDeltaTranslation(deltaY, 3));
		deltaYList.set(2, getDeltaTranslation(deltaY, 5));
		deltaYList.set(3, getDeltaTranslation(deltaY, 7));

		magList.set(0, magnitude(deltaYList.get(0), deltaXList.get(0)));
		magList.set(1, magnitude(deltaYList.get(1), deltaXList.get(1)));
		magList.set(2, magnitude(deltaYList.get(2), deltaXList.get(2)));
		magList.set(3, magnitude(deltaYList.get(3), deltaXList.get(3)));

		dirList.set(0, atan2(deltaYList.get(0), deltaXList.get(0)));
		dirList.set(1, atan2(deltaYList.get(1), deltaXList.get(1)));
		dirList.set(2, atan2(deltaYList.get(2), deltaXList.get(2)));
		dirList.set(3, atan2(deltaYList.get(3), deltaXList.get(3)));

		double magMax = Collections.max(magList);

		m_driveTrain.driveFL.set(magList.get(0) * mag / magMax);
		m_driveTrain.driveFL.set(magList.get(1) * mag / magMax);
		m_driveTrain.driveFL.set(magList.get(2) * mag / magMax);
		m_driveTrain.driveFL.set(magList.get(3) * mag / magMax);

		setSteering(m_driveTrain.steerFL, dirList.get(0));
		setSteering(m_driveTrain.steerBL, dirList.get(1));
		setSteering(m_driveTrain.steerBR, dirList.get(2));
		setSteering(m_driveTrain.steerFR, dirList.get(3));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	private double magnitude(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) +
						 Math.pow(y, 2));
	}

	private double getDeltaTranslation(double deltaTranslation, double motorHeadingOffsetScale) {
		return deltaTranslation + motorToCenterDistance * Math.cos(currentHeading + deltaHeading + motorHeadingOffsetScale / 2 * motorHeadingOffset);
	}

	private double atan2(double y, double x) {
		double temp = Math.atan2(y, x);
		if (temp < 0) {
			return temp + 2 * Math.PI;
		}
		else return temp;
	}

	private double getDeltaHeading(double currentHeading, double targetHeading, double strength) {
		double currentHeadingMod = currentHeading % (2 * Math.PI);

		if (targetHeading > currentHeadingMod) {
			if ((targetHeading - currentHeadingMod) < Math.PI) {
				return Math.min((targetHeading - currentHeadingMod), maxTurnRate) * strength + currentHeading;
			}
			else {
				return Math.min(2 * Math.PI - (targetHeading - currentHeadingMod), maxTurnRate) * -1 * strength + currentHeading;
			}
		}
		else {
			if ((currentHeadingMod - targetHeading) < Math.PI) {
				return Math.min((currentHeadingMod - targetHeading), maxTurnRate) * -1 * strength + currentHeading;
			}
			else {
				return Math.min(2 * Math.PI - (currentHeadingMod - targetHeading), maxTurnRate) * strength + currentHeading;
			}
		}
	}

	private void setSteering(WPI_TalonSRX motor, double position) {
	}
}
