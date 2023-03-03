package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.Top2Piece;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class Top2PieceAuto extends SequentialCommandGroup{
    public Top2PieceAuto(PathFollowingSwerve swerve, Arm arm) {
        addCommands(
            new Top2Piece(swerve, arm)
        );
    }
    
}
