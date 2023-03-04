package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.PlaceAndMove;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class BasicAuto extends SequentialCommandGroup{
    public BasicAuto(PathFollowingSwerve swerve, Arm arm, Intake intake){
        addCommands(
            new PlaceAndMove(swerve, arm, intake)
        );
    }
}