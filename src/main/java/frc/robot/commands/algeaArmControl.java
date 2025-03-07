package frc.robot.commands;

import frc.robot.subsystems.algaeArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class algeaArmControl extends Command {
	private final algaeArm m_Algearm;
	private DoubleSupplier trigger;

	public algeaArmControl(algaeArm algearm, XboxController xbox){
		m_Algearm = algearm;
		addRequirements(m_Algearm);

		trigger = xbox::getRightTriggerAxis;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double position = 0 - 38 * trigger.getAsDouble();
		m_Algearm.updateTargetAngle(position);

	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
