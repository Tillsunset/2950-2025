package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private SendableChooser<Command> m_chooser = new SendableChooser<>();

	private CommandXboxController green = new CommandXboxController(0);
	private Trigger dDown = green.povDown();
	private Trigger dLeft = green.povLeft();
	private Trigger dUp = green.povUp();
	private Trigger dRight = green.povRight();
    private Trigger Abutton = green.a();
	private Trigger Bbuttton = green.b();
	private Trigger Xbutton = green.x();
	private Trigger Ybutton = green.y();

	private CommandXboxController purple = new CommandXboxController(1);
	private Trigger Rbbutton = purple.rightBumper();
	private Trigger Lbbutton = purple.leftBumper();
	// private Trigger ltTrigger = purple.leftTrigger(); //used for front flip
	// private Trigger rTrigger = purple.rightTrigger(); //used for AlgaeArm

	private driveTrain m_driveTrain = new driveTrain();
	private elevator m_elevator = new elevator();
	private coralIntake m_coralIntake = new coralIntake();
	private winch m_Winch = new winch();
	private algaeArm m_AlgeaArm = new algaeArm();
	private algaeIntake m_algeaIntake = new algaeIntake();
	
	private driveTank m_driveTank = new driveTank(m_driveTrain, purple.getHID());
	// private driveArcade m_driveArcade = new driveArcade(m_driveTrain, green.getHID());
	// private driveCheesy m_driveCheesy = new driveCheesy(m_driveTrain, green.getHID());

	private elevatorL1 m_elevatorL1 = new elevatorL1(m_elevator);
	private elevatorL2 m_elevatorL2 = new elevatorL2(m_elevator);
	private elevatorL3 m_elevatorL3 = new elevatorL3(m_elevator);
	private elevatorFeed m_elevatorFeed = new elevatorFeed(m_elevator);

	private coralIn m_coralIn = new coralIn(m_coralIntake);
	private coralOut m_coralOut = new coralOut(m_coralIntake);
	private algeaArmControl m_AlgeaArmControl = new algeaArmControl(m_AlgeaArm, purple.getHID());

	private algeaIn m_AlgeaIn = new algeaIn(m_algeaIntake);
	private algeaOut m_AlgeaOut = new algeaOut(m_algeaIntake);

	private winchUp m_WinchUp = new winchUp(m_Winch);
	private winchDown m_WinchDown = new winchDown(m_Winch);

	private leaveStartingLine m_leave = new leaveStartingLine(m_driveTrain);
	private alignUsingAprilTag m_align = new alignUsingAprilTag(m_driveTrain);

	private Command autoL3 = m_leave
	.andThen(Commands.parallel(m_align, m_elevatorL3))
	.andThen(m_coralIn)
	.andThen(m_elevatorL1);

	public RobotContainer() {
		m_chooser.setDefaultOption("nothing", null);
		m_chooser.addOption("Leave Starting", m_leave);
		// m_chooser.addOption("auto Score L3", autoL3);

		m_driveTrain.setDefaultCommand(m_driveTank);
		// m_driveTrain.setDefaultCommand(m_driveArcade);
		// m_driveTrain.setDefaultCommand(m_driveCheesy);

		m_AlgeaArm.setDefaultCommand(m_AlgeaArmControl);

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

		Lbbutton.whileTrue(m_AlgeaIn);
		Rbbutton.whileTrue(m_AlgeaOut);
	}

	public Command getAutonomousCommand() {
	    return m_chooser.getSelected();
	}
}
