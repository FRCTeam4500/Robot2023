package frc.robot.subsystem.swerve.command;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.subsystem.swerve.Swerve;

public class BalanceCommand extends CommandBase {
	private Swerve swerve;
  	private double tolerence = 2;
	private boolean balanced = false;
	private int balanceCounter = 0;
	private final int MAX_COUNT = 10; // Ends after 10 cycles of being balanced
	private AHRSAngleGetterComponent gyro = swerve.getGyro();

	/**
	 * Creates a new BalanceCommand.
	 *
	 * @param swerve The subsystem used by this command.
	 */
	public BalanceCommand(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve.resetRobotAngle();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		if (gyro.getPitch() < -tolerence) {
			swerve.moveFieldCentric(-0.3, 0, 0);
			balanced = false;
			balanceCounter = 0;

		} else if (gyro.getPitch() > tolerence) {
			swerve.moveFieldCentric(0.3, 0, 0);
			balanced = false;
			balanceCounter = 0;
			
		} else {
			swerve.moveFieldCentric(0, 0, 0);
			if (balanceCounter < MAX_COUNT) {
				balanceCounter++;
			} else {
				balanced = true;
			}
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		swerve.moveFieldCentric(0, 0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (balanced) {
			return true;
		}
		return false;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Gyro Pitch", gyro::getPitch, null);
		builder.addBooleanProperty("Is Balanced", () -> balanced, null);
	}
}
