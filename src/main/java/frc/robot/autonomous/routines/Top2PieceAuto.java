package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class Top2PieceAuto extends SequentialCommandGroup{
    public Top2PieceAuto(PathFollowingSwerve swerve) {
        addCommands(
            new Top2PieceAuto(swerve)
        );
    }
    
}
