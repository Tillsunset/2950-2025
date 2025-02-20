/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.algeaIntake;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An example command that uses an example subsystem.
 */
public class algeaIn extends Command {
	private final algeaIntake m_algeaIntake;
	// AnalogInput analog = new AnalogInput(3);

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public algeaIn(algeaIntake algeaIntake){
		// Use requires() here to declare subsystem dependencies.
		m_algeaIntake = algeaIntake;
		addRequirements(m_algeaIntake);

		// analog.setAverageBits(3);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_algeaIntake.setOutput(1.);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// System.out.println(analog.getAverageVoltage()/(0.293));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_algeaIntake.setOutput(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
