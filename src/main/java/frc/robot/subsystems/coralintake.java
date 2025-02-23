package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralIntake extends SubsystemBase {

	private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(10);

	public coralIntake() {	
	}

	public void setOutput(double output) {
		intakeMotor.set(output);
	}
}