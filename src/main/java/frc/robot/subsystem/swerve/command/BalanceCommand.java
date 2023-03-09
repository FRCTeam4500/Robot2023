package frc.robot.subsystem.swerve.command;

import com.fasterxml.jackson.databind.deser.impl.ExternalTypeHandler.Builder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;

public class BalanceCommand extends CommandBase {
    private Swerve swerve;
    private double tolerance = 2.0; // degrees
    private double maxSpeed = 1; // meters per second
    private boolean facingChargingStation;
    private AHRSAngleGetterComponent gyro = HardwareSwerveFactory.getGyro();
    private double currentPitch;
    private int MAX_TIMES = 600;

    /**
     * Creates a new BalanceCommand.
     *
     * @param swerve The subsystem used by this command.
     * @param facingChargingStation Whether the robot is facing the charging station.
     */
    public BalanceCommand(Swerve swerve, boolean facingChargingStation) {
        this.swerve = swerve;
        this.facingChargingStation = facingChargingStation;

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.moveRobotCentric((facingChargingStation ? maxSpeed : -maxSpeed), 0.0, 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentPitch = Math.toDegrees(gyro.getPitch());

        if (currentPitch > tolerance) {
            swerve.moveRobotCentric(maxSpeed, 0.0, 0.0);
        } else if (currentPitch < -tolerance) {
            swerve.moveRobotCentric(-maxSpeed, 0.0, 0.0);
        } else {
            swerve.moveRobotCentric(0.0, 0.0, 0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.moveFieldCentric(0.0, 0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (Math.abs(currentPitch) < tolerance) {
        //     MAX_TIMES--;
        //     if (MAX_TIMES <= 0) {
        //         return true;
        //     }
        // } else {
        //     MAX_TIMES = 50;
        // }
        // return false;
        return false;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current Balance Pitch", () -> currentPitch, null);
        builder.addDoubleProperty("Max Times", () -> MAX_TIMES, null);
    }
}
