package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;

public class ShootCommand extends Command {
	private Shooter shooter;
	private Angle shooterAngle;
	private double launchVolts = 10.0;
	private double launchSpeed = 20.0;
	private double kickerSpeed = 0.1;

	public ShootCommand(Shooter shooter, Angle angle) {
		this.shooter = shooter;
		shooter.setLaunchVoltage(0.0);
		shooter.launchSpeed = launchSpeed;
		shooterAngle = angle;
	}

	public void initialize() {
		shooter.setShooterAngle(shooterAngle);
	}

	@Override
	public void execute() {
		shooter.setLaunchVoltage(launchVolts);

		if (shooter.atLaunchSpeed()) {
			shooter.setKickerSpeed(kickerSpeed);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setLaunchVoltage(0.0);
		shooter.setKickerSpeed(0.0);
		shooter.setShooterCoastOut();
	}
}
