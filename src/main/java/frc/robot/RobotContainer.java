// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private CommandXboxController xbox = new CommandXboxController(0);

	private driveTrain m_driveTrain = new driveTrain();
	
	private driveWithJoystickFOC m_driveWithJoystick = new driveWithJoystickFOC(m_driveTrain, xbox.getHID());

	public RobotContainer() {
		m_driveTrain.setDefaultCommand(m_driveWithJoystick);
		configureBindings();
	}
	
	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
