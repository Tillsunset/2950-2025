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
    private Trigger Abutton = xbox.a();
	private Trigger Bbuttton = xbox.b();
	private Trigger Xbutton = xbox.x();
	private Trigger Ybutton = xbox.y();

	private driveTrain m_driveTrain = new driveTrain();
	private elevator m_elevator = new elevator();
	private coralIntake m_coralIntake = new coralIntake();
	private winch m_Winch = new winch();
	// private algeaArm m_AlgeaArm = new algeaArm();
	
	private driveWithJoystick m_driveWithJoystick = new driveWithJoystick(m_driveTrain, xbox.getHID());
	// private driveWithAprilTag m_driveWithAprilTag = new driveWithAprilTag(m_driveTrain);

	private elevatorL1 m_elevatorL1 = new elevatorL1(m_elevator);
	private elevatorL2 m_elevatorL2 = new elevatorL2(m_elevator);
	private elevatorL3 m_elevatorL3 = new elevatorL3(m_elevator);
	private elevatorFeed m_elevatorFeed = new elevatorFeed(m_elevator);

	private coralIn m_coralIn = new coralIn(m_coralIntake);
	private coralOut m_coralOut = new coralOut(m_coralIntake);

	// private algeaArmDown m_AlgeaArmDown = new algeaArmDown(m_AlgeaArm);
	// private algeaArmUp m_AlgeaarmUp = new algeaArmUp(m_AlgeaArm);

	private winchUp m_WinchUp = new winchUp(m_Winch);
	private winchDown m_WinchDown = new winchDown(m_Winch);

	public RobotContainer() {
		m_driveTrain.setDefaultCommand(m_driveWithJoystick);
		configureBindings();
	}
	
	private void configureBindings() {
		dDown.whileTrue(m_elevatorL1);
		dLeft.whileTrue(m_elevatorL2);
		dUp.whileTrue(m_elevatorL3);
		dRight.whileTrue(m_elevatorFeed);

		Abutton.whileTrue(m_coralIn);
		Bbuttton.whileTrue(m_coralOut);

		Xbutton.whileTrue(m_WinchUp);
		Ybutton.whileTrue(m_WinchDown);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
