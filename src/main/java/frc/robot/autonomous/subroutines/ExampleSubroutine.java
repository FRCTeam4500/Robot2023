
package frc.robot.autonomous.subroutines;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import frc.robot.RobotContainer;

public class ExampleSubroutine extends SequentialCommandGroup {
    public ExampleSubroutine (PathFollowingSwerve swerve) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("ExamplePathWeaverFile");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);
        RobotContainer.isCone = true;
        RobotContainer.isBottomCone = false;
        
        addCommands(
            new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
            RobotContainer.COMMAND_MAP.get("Ready Top"),
            RobotContainer.COMMAND_MAP.get("Place"),
            new WaitCommand(0.5),
            RobotContainer.COMMAND_MAP.get("Zero"),
            swerveCmd
        );
    }
}


