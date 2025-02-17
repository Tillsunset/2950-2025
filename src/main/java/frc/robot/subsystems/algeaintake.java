package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algeaIntake extends SubsystemBase {

	private WPI_VictorSPX algeaIntake = new WPI_VictorSPX(1);

	public algeaIntake() {
	} 
	
	public void set(double output) {
		algeaIntake.set(output);
	}
}
