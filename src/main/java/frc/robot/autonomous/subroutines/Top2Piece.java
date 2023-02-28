
package frc.robot.autonomous.subroutines;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import frc.robot.RobotContainer;

public class Top2Piece extends SequentialCommandGroup {
    public Top2Piece (PathFollowingSwerve swerve) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("A1");
        Trajectory path2 = ExtendedTrajectoryUtilities.getDeployedTrajectory("1A");
        Trajectory turnPath = ExtendedTrajectoryUtilities.getDeployedTrajectory("Turn");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);
        Command swerveCmd2 = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path2);
        Command swerveCmd3 = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, turnPath);
        
        addCommands(
            new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
            swerveCmd3,
            // Go into intaking position
            new WaitCommand(0.5),
            swerveCmd,
            new WaitCommand(.1),
            swerveCmd2,
            new InstantCommand(() -> swerve.moveFieldCentric(0, 0, 0))

        );
    }
}


