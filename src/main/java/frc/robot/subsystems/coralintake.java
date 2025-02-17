package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralIntake extends SubsystemBase {

	private WPI_VictorSPX coralIntake = new WPI_VictorSPX(1);

	public coralIntake() {	
	}

	public void set(double output) {
		coralIntake.set(output);
	}
}