package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntake extends SubsystemBase {

	private WPI_VictorSPX algeaIntake = new WPI_VictorSPX(1);

	public algaeIntake() {
		algeaIntake.setNeutralMode(NeutralMode.Brake);
	} 
	
	public void setOutput(double output) {
		algeaIntake.set(output);
	}
}
