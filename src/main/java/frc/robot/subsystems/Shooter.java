package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

public class Shooter extends Command {
	private final SparkMax axleMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
	private final SparkMax pulleyMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
	private final TalonFX launchMotor = new TalonFX(0);
	private final TalonFX angleMotor = new TalonFX(11);
	public static final double angleGearRatio =  1764d / 180d ;
	private final Angle restingAngle = Degrees.of(57.0);

	private double launchVoltage;
	public double launchSpeed;

	public Shooter() {
		launchVoltage = 1.0;
		launchSpeed = 80.0;

		var talonFXconfigs = new TalonFXConfiguration();

		Slot0Configs config = talonFXconfigs.Slot0;
		config.kP = 20.0;
		config.kI = 0.0;
		config.kD = 0.0;
		config.kG = 0.1;
		config.GravityType = GravityTypeValue.Arm_Cosine;

		FeedbackConfigs feedback = talonFXconfigs.Feedback;
		feedback.RotorToSensorRatio = 1;
		feedback.SensorToMechanismRatio = angleGearRatio;

		talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		angleMotor.getConfigurator().apply(talonFXconfigs);

		angleMotor.setPosition(restingAngle);

	}

	public void setShooterAngle(Angle angle) {
		angleMotor.setControl(new PositionVoltage(angle));
	}

	public void setShooterCoastOut(){

		angleMotor.setControl(new CoastOut());
	}

	public void setKickerSpeed(double speed) {
		axleMotor.set(speed);
		pulleyMotor.set(speed);
	}

	public void setLaunchVoltage(double volts) {
		launchVoltage = volts;
		launchMotor.setVoltage(volts);
	}

	public boolean atLaunchSpeed() {
		return launchMotor.getVelocity().getValue().in(Units.RotationsPerSecond) >= launchSpeed;
	}

}
