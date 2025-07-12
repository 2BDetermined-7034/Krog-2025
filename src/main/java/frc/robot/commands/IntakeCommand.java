package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;

public class IntakeCommand extends Command {
	private Shooter shooter;
	private Angle shooterAngle;
	private double launchVolts = -3.0;
	private double kickerSpeed = 0.3;

	public IntakeCommand(Shooter shooter) {
		this.shooter = shooter;
		shooter.setLaunchVoltage(0.0);
		shooterAngle = Degrees.of(55.0);
	}

	public void initialize() {
		shooter.setShooterAngle(shooterAngle);
	}

	@Override
	public void execute() {
		shooter.setLaunchVoltage(launchVolts);
		shooter.setKickerSpeed(-0.3);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setLaunchVoltage(0.0);
		shooter.setKickerSpeed(0.0);
		shooter.setShooterCoastOut();
	}
}
