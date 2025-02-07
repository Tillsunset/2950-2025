package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class archerarm extends SubsystemBase {

private SparkMax archer1 = new SparkMax(1, MotorType.kBrushless);

 public archerarm(){
   SparkMaxConfig archerarMaxConfig = new SparkMaxConfig();
    archerarMaxConfig
   .smartCurrentLimit(20)
			.idleMode(IdleMode.kBrake);
            archer1.configure(archerarMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
 } 
}