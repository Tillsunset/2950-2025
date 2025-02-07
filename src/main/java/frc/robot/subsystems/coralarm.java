package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralarm extends SubsystemBase {

private SparkMax coralarm1 = new SparkMax(1, MotorType.kBrushless);

 public coralarm(){
   SparkMaxConfig coralMaxConfig = new SparkMaxConfig();
   coralMaxConfig
   .smartCurrentLimit(20)
			.idleMode(IdleMode.kBrake);
            coralarm1.configure(coralMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
 } 
}