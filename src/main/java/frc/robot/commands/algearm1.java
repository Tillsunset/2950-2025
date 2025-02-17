package frc.robot.commands;

import frc.robot.subsystems.algeaArm;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * An example command that uses an example subsystem.
 */
public class algearm1 extends Command {
	private final algeaArm m_Algearm;


	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public algearm1(algeaArm algearm){
		// Use requires() here to declare subsystem dependencies.
		m_Algearm = algearm;
		addRequirements(m_Algearm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        m_Algearm.updateTargetAngle(5);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
