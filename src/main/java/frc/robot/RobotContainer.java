// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private CommandXboxController xbox = new CommandXboxController(0);
	private Trigger buttonRB = xbox.rightBumper();
	private Trigger dDown = xbox.povDown();
	private Trigger dLeft = xbox.povLeft();
	private Trigger dUp = xbox.povUp();
	private Trigger dRight = xbox.povRight();

	private driveTrain m_driveTrain = new driveTrain();
	// private elevator m_elevator = new elevator();
	
	private driveWithJoystick m_driveWithJoystick = new driveWithJoystick(m_driveTrain, xbox.getHID());
	private driveWithAprilTag m_driveWithAprilTag = new driveWithAprilTag(m_driveTrain);
	// private elevatorL1 m_elevatorL1 = new elevatorL1(m_elevator);
	// private elevatorL2 m_elevatorL2 = new elevatorL2(m_elevator);

	public RobotContainer() {
		m_driveTrain.setDefaultCommand(m_driveWithJoystick);
		// m_elevator.register();
		configureBindings();
	}
	
	private void configureBindings() {
		buttonRB.whileTrue(m_driveWithAprilTag);
		// dDown.whileTrue(m_elevatorL1);
		// dLeft.whileTrue(m_elevatorL2);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
