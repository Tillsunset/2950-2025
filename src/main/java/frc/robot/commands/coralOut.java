/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.coralIntake;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An example command that uses an example subsystem.
 */
public class coralOut extends Command {
	private final coralIntake m_coralIntake;
	// AnalogInput analog = new AnalogInput(3);

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public coralOut(coralIntake coralintake){
		// Use requires() here to declare subsystem dependencies.
		m_coralIntake = coralintake;
		addRequirements(m_coralIntake);

		// analog.setAverageBits(3);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_coralIntake.setOutput(-1.);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// System.out.println(analog.getAverageVoltage()/(0.293));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_coralIntake.setOutput(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
