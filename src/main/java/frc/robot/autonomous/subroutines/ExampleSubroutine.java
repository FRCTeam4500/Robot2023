/*
package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystem.intake.IntakeSetAngleCommand;
import frc.robot.subsystem.intake.IntakeSetOutputCommand;
import frc.robot.subsystem.arm.ArmSetTiltAngleCommand;
import frc.robot.subsystem.arm.ArmSetWinchOutputCommand;

public class ExampleSubroutine extends SequentialCommandGroup {
    public ExampleSubroutine (PathFollowingSwerve swerve, Arm arm, Intake intake) {
        addRequirments(swerve, arm, intake);
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("ExamplePathWeaverFile");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);
        
        addCommands(
            new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
            new ParallelCommandGroup(
                // TODO: Align with AprilTag First 
                new ArmSetTiltAngleCommand(arm, ArmConstants.ARM_PLACE_ANGLE),
                new ArmSetWinchOutputCommand(arm, ArmConstants.ARM_PLACE_TOP),
            )
        )
    }
}
*/

