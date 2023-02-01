package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystem.Arm.Position;

public class SubsystemSequencer {
    
    public ParallelCommandGroup ArmIntakeCommand(Position position, Arm arm, Intake intake) {
        switch (position) {
            case Bottom:
                return new ParallelCommandGroup(
                    new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_BOTTOM_TILT_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_BOTTOM_WINCH_OUTPUT)),
                    new Intake.IntakeSetAngleCommand(intake, IntakeConstants.INTAKE_ANGLE_BOTTOM)
            );
            case Middle:
                return new ParallelCommandGroup(
                    new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_MIDDLE_TILT_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_MIDDLE_WINCH_OUTPUT)),
                    new Intake.IntakeSetAngleCommand(intake, IntakeConstants.INTAKE_MIDDLE_ANGLE)
            );
            case Top:
                return new ParallelCommandGroup(
                    new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_TOP_TILT_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_TOP_WINCH_OUTPUT)),
                    new Intake.IntakeSetAngleCommand(intake, IntakeConstants.INTAKE_TOP_ANGLE)
            );
            case Retracted:
                return new ParallelCommandGroup(
                    new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_RETRACTED_TILT_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_RETRACTED_WINCH_OUTPUT)),
                    new Intake.IntakeSetAngleCommand(intake, IntakeConstants.INTAKE_RETRACTED_ANGLE)
            );
            case Ground:
                return new ParallelCommandGroup(
                    new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_GROUND_TILT_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_GROUND_WINCH_OUTPUT)),
                    new Intake.IntakeSetAngleCommand(intake, IntakeConstants.INTAKE_GROUND_ANGLE)
            );
            default:
                return null;
        }
    }
    
}
