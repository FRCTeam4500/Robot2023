package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import static frc.robot.RobotContainer.isCone;

public class PlaceAndMove extends SequentialCommandGroup{
    public PlaceAndMove(PathFollowingSwerve swerve, Arm arm, Intake intake){
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("Curve");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);

        addCommands(

            new InstantCommand(() -> {swerve.resetRobotAngle();}),
            new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
            new InstantCommand(() -> isCone = true),
            new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_ZERO_ANGLE),
            new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_RETRACT),
            new WaitCommand(.5),
            new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_LAUNCH_ANGLE),
            new WaitCommand(.5),
            new Arm.ArmSetWinchOutputCommand(arm, ArmConstants.ARM_PLACE_TOP),
            new Intake.IntakeSetAngleCommand(intake, true, true, false, false),
            new WaitCommand(1.5),
            new Intake.IntakeSetOutputCommand(intake, false, true),
            new WaitCommand(1),
            new Intake.IntakeSetOutputCommand(intake, true, false),
            new Intake.IntakeSetAngleCommand(intake, false, false, true, false),
            new Arm.ArmSetWinchOutputCommand(arm, 0),
            new Arm.ArmSetTiltAngleCommand(arm, ArmConstants.ARM_ZERO_ANGLE),
            new WaitCommand(.5),
            swerveCmd,
            new InstantCommand(() -> swerve.moveFieldCentric(0, 0, 0))
        );
    }       
}
