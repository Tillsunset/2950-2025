package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private SendableChooser<Command> m_chooser = new SendableChooser<>();

	private CommandXboxController adaptive = new CommandXboxController(0);
	private Trigger adaptiveDown = adaptive.povDown();
	private Trigger adaptiveLeft = adaptive.povLeft();
	private Trigger adaptiveUp = adaptive.povUp();
	private Trigger adaptiveRight = adaptive.povRight();
    private Trigger adaptiveA = adaptive.a();
	private Trigger adaptiveB = adaptive.b();
	private Trigger adaptiveStart = adaptive.x();
	private Trigger adaptiveSelect = adaptive.y();

	private CommandXboxController green = new CommandXboxController(1);
	private Trigger greenRB = green.rightBumper();
	private Trigger greenLB = green.leftBumper();
	// private Trigger ltTrigger = green.leftTrigger(); //used for front flip
	// private Trigger rTrigger = green.rightTrigger(); //used for AlgaeArm

	private driveTrain m_driveTrain = new driveTrain();
	private coralIntake m_coralIntake = new coralIntake();
	private algaeIntake m_algeaIntake = new algaeIntake();
	private algaeArm m_algeaArm = new algaeArm();
	private elevator m_elevator = new elevator();
	private winch m_Winch = new winch();
	private empty m_empty = new empty();
	
	private driveTank m_driveTank = new driveTank(m_driveTrain, green.getHID());

	private elevatorL1 m_elevatorL1 = new elevatorL1(m_elevator);
	private elevatorL2 m_elevatorL2 = new elevatorL2(m_elevator);
	private elevatorL3 m_elevatorL3 = new elevatorL3(m_elevator);
	private elevatorFeed m_elevatorFeed = new elevatorFeed(m_elevator);

	private coralIn m_coralIn = new coralIn(m_coralIntake);
	private coralOut m_coralOut = new coralOut(m_coralIntake);

	private algeaArmControl m_algeaArmControl = new algeaArmControl(m_algeaArm, green.getHID());
	
	private algeaIn m_AlgeaIn = new algeaIn(m_algeaIntake);
	private algeaOut m_AlgeaOut = new algeaOut(m_algeaIntake);

	private winchUp m_WinchUp = new winchUp(m_Winch);
	private winchDown m_WinchDown = new winchDown(m_Winch);

	private leaveStartingLine m_leave = new leaveStartingLine(m_driveTrain);

	private alignUsingAprilTag m_align = new alignUsingAprilTag(m_empty);

	public RobotContainer() {
		m_chooser.setDefaultOption("nothing", null);
		m_chooser.addOption("Leave Starting", m_leave);
		// m_chooser.addOption("Leave Score L2", m_LeaveScoreL2);
		// m_chooser.addOption("Leave Score L3", m_LeaveScoreL3);

		SmartDashboard.putData("Auto choices", m_chooser);

		m_driveTrain.setDefaultCommand(m_driveTank);
		m_algeaArm.setDefaultCommand(m_algeaArmControl);
		m_empty.setDefaultCommand(m_align);

		configureBindings();
	}
	
	private void configureBindings() {
		adaptiveDown.whileTrue(m_elevatorL1);
		adaptiveLeft.whileTrue(m_elevatorL2);
		adaptiveUp.whileTrue(m_elevatorL3);
		adaptiveRight.whileTrue(m_elevatorFeed);

		adaptiveA.whileTrue(m_coralIn);
		adaptiveB.whileTrue(m_coralOut);

		adaptiveStart.whileTrue(m_WinchUp);
		adaptiveSelect.whileTrue(m_WinchDown);

		greenLB.whileTrue(m_AlgeaIn);
		greenRB.whileTrue(m_AlgeaOut);
	}

	public Command getAutonomousCommand() {
	    return m_chooser.getSelected();
	}
}
