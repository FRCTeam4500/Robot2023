
package frc.robot.autonomous.subroutines;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Top2Piece extends SequentialCommandGroup {
    public Top2Piece (PathFollowingSwerve swerve, Arm arm) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("A1");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);
        
        addCommands(
            new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
            new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_ZERO_ANGLE),
            new WaitCommand(3),
            swerveCmd,
            new InstantCommand(() -> swerve.moveFieldCentric(0, 0, 0))

        );
    }
}


