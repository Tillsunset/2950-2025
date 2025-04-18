package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private SendableChooser<Command> m_chooser = new SendableChooser<>();

	private CommandXboxController adaptive = new CommandXboxController(0);
	private Trigger adaptiveDown = 				adaptive.povDown();
	private Trigger adaptiveLeft = 				adaptive.povLeft();
	private Trigger adaptiveUp = 				adaptive.povUp();
	private Trigger adaptiveRight = 			adaptive.povRight();
    private Trigger adaptiveA = 				adaptive.a();
	private Trigger adaptiveB = 				adaptive.b();
	private Trigger adaptiveStart = 				adaptive.start();
	private Trigger adaptiveSelect = 				adaptive.back();

	private CommandXboxController green = new CommandXboxController(1);
	private Trigger greenRB = 					green.rightBumper();
	private Trigger greenLB = 					green.leftBumper();
	private Trigger greenA = 					green.a();
	private Trigger greenB = 					green.b();
	private Trigger greenX = 					green.x();
	// private Trigger ltTrigger = purple.leftTrigger(); //used for front flip
	// private Trigger rTrigger = purple.rightTrigger(); //used for AlgaeArm

	private driveTrain m_driveTrain = new driveTrain();
	private coralIntake m_coralIntake = new coralIntake();
	private algaeIntake m_algeaIntake = new algaeIntake();
	private algaeArm m_algeaArm = new algaeArm();
	private elevator m_elevator = new elevator();
	// private winch m_Winch = new winch();
	
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

	// private winchUp m_WinchUp = new winchUp(m_Winch);
	// private winchDown m_WinchDown = new winchDown(m_Winch);

	private leaveStarting m_leave = new leaveStarting(m_driveTrain);
	private SequentialCommandGroup m_leaveScoreL2 = new leaveStarting(m_driveTrain).andThen(
													new elevatorL2(m_elevator)).andThen(
													new AprilStanleyTest(m_driveTrain)).andThen(
													new coralShoot(m_coralIntake)).andThen(
													new elevatorL3(m_elevator)).andThen(
													new backUp(m_driveTrain)
													);
	private SequentialCommandGroup m_pauseScoreL2 = new WaitCommand(5.0).andThen(
													new leaveStarting(m_driveTrain)).andThen(
													new elevatorL2(m_elevator)).andThen(
													new AprilStanleyTest(m_driveTrain)).andThen(
													new coralShoot(m_coralIntake)).andThen(
													new elevatorL3(m_elevator)).andThen(
													new backUp(m_driveTrain)
													);

	private SequentialCommandGroup m_leaveScoreL3 = new leaveStarting(m_driveTrain).andThen(
													new elevatorL3(m_elevator)).andThen(
													new AprilStanleyTest(m_driveTrain)).andThen(
													new coralShoot(m_coralIntake)).andThen(
													new elevatorL1(m_elevator)).andThen(
													new backUp(m_driveTrain));

	private SequentialCommandGroup m_pauseScoreL3 = new WaitCommand(5.0).andThen(
													new leaveStarting(m_driveTrain)).andThen(
													new elevatorL3(m_elevator)).andThen(
													new AprilStanleyTest(m_driveTrain)).andThen(
													new coralShoot(m_coralIntake)).andThen(
													new elevatorL1(m_elevator)).andThen(
													new backUp(m_driveTrain));

	// private leaveScoreL3 m_leaveScoreL3 = new leaveScoreL3(m_driveTrain, m_elevator, m_coralIntake);
	// private leaveScoreL2 m_leaveScoreL2 = new leaveScoreL2(m_driveTrain, m_elevator, m_coralIntake);

	private AprilTest m_april = new AprilTest(m_driveTrain);
	private StanleyTest m_stanley = new StanleyTest(m_driveTrain);
	private AprilStanleyTest m_aprilStanley = new AprilStanleyTest(m_driveTrain);

	public RobotContainer() {
		m_chooser.setDefaultOption("nothing", null);
		m_chooser.addOption("Leave Starting", m_leave);
		m_chooser.addOption("Leave Score L2", m_leaveScoreL2);
		m_chooser.addOption("Pause 5 Sec, Score L2", m_pauseScoreL2);
		m_chooser.addOption("Leave Score L3", m_leaveScoreL3);
		m_chooser.addOption("Pause 5 Sec, Score L3", m_pauseScoreL3);

		SmartDashboard.putData("Auto choices", m_chooser);

		m_driveTrain.setDefaultCommand(m_driveTank);
		m_algeaArm.setDefaultCommand(m_algeaArmControl);

		configureBindings();
	}
	
	private void configureBindings() {
		adaptiveDown.whileTrue(m_elevatorL1);
		adaptiveLeft.whileTrue(m_elevatorL2);
		adaptiveUp.whileTrue(m_elevatorL3);
		adaptiveRight.whileTrue(m_elevatorFeed);

		adaptiveA.whileTrue(m_coralIn);
		adaptiveB.whileTrue(m_coralOut);

		// adaptiveStart.whileTrue(m_WinchUp);
		// adaptiveSelect.whileTrue(m_WinchDown);

		greenLB.whileTrue(m_AlgeaIn);
		greenRB.whileTrue(m_AlgeaOut);

		greenA.whileTrue(m_april);
		greenB.whileTrue(m_stanley);
		greenX.whileTrue(m_aprilStanley);
	}

	public Command getAutonomousCommand() {
	    return m_chooser.getSelected();
	}
}
