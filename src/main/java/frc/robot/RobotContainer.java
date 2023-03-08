package frc.robot;


import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;
import static frc.robot.Constants.RobotConstants.commandMap;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystem.Arm.Position;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.subsystem.swerve.command.BalanceCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.vision.HardwareVisionFactory;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand.ControlMode;


public class RobotContainer {
    /* Setting Joystick Buttons */
    private final Joystick driveStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);
    private final ControllerInfo info = new ControllerInfo();

    private final JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, JoystickConstants.LOCK_SWERVE_ROTATION);
    private final JoystickButton switchDriveModeRobotCentricButton = new JoystickButton(driveStick, JoystickConstants.SWITCH_DRIVE_MODE_ROBOT_CENTRIC);
    private final JoystickButton alignSwerveToAngleButton = new JoystickButton(driveStick, JoystickConstants.ALIGN_SWERVE_TO_ANGLE);
    private final JoystickButton alignSwerveReverseButton = new JoystickButton(driveStick, JoystickConstants.ALIGN_SWERVE_REVERSE);
    private final JoystickButton resetGyroButton = new JoystickButton(driveStick, JoystickConstants.RESET_GYRO);
    private final JoystickButton limitSwerveSpeedButton = new JoystickButton(driveStick, JoystickConstants.LIMIT_SWERVE_SPEED);
    private final JoystickButton noForwardButton = new JoystickButton(driveStick, JoystickConstants.NO_FORWARD);

    private final JoystickButton balanceButton = new JoystickButton(controlStick, 5);

    private final JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private final JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private final JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private final JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private final JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private final JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    private final JoystickButton sidewaysConeButton = new JoystickButton(controlStick, JoystickConstants.SIDEWAYS_CONE);
    private final JoystickButton readySubstationButton = new JoystickButton(controlStick, JoystickConstants.SUBSTATION_PICKUP);

    private final JoystickButton tiltUp = new JoystickButton(controlStick, 4);
    private final JoystickButton tiltDown = new JoystickButton(controlStick, 2);
    private final DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private TriModeSwerveCommand swerveCommand;
    private BalanceCommand balanceCommand;
    public static boolean isCone = true; // Changes with coneButton/cubeButton
    public static boolean isBottomCone = true; // Changes with Orientation buttons

    private final PathFollowingSwerve m_swerve = HardwareSwerveFactory.makeSwerve();
    private final Arm m_arm = makeArm();
    private final Intake m_intake = makeIntake();
    private final Vision m_vision = HardwareVisionFactory.makeVision();

    /**Both PID constants need to be tested */
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_swerve::getCurrentPose, m_swerve::resetPose, new PIDConstants(5, 0, 0), new PIDConstants(4, 0, 0), m_swerve::moveRobotCentric, commandMap, m_swerve);    
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();
    

    public RobotContainer() {
        configureControls();
        configureCommands();
        configureSwerve();
        configureAuto();
        configureArmAndIntake();
    }

    void configureControls() {
        info.xSensitivity = 4;
        info.ySensitivity = 4;
        info.zSensitivity = 3.5;
        info.xDeadzone = 0.2;
        info.yDeadzone = 0.2;
        info.zDeadzone = 0.2;
        Shuffleboard.getTab("Driver Controls").add("Driver Controls", info);
        Shuffleboard.getTab("Driver Controls").add("Messages", messages);
    }

    
    
    void configureSwerve() {
        swerveCommand = new TriModeSwerveCommand(m_swerve, driveStick, info, messages);
        balanceCommand = new BalanceCommand(m_swerve, true);
        m_swerve.setDefaultCommand(swerveCommand);

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentricButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;}));
        switchDriveModeRobotCentricButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        /* Slow Side to Side movement */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
        Shuffleboard.getTab("Swerve").add("Balance Command", balanceCommand);
    }

    void configureCommands() {
        commandMap.put(
            "readyBot", 
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PICKUP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOT_ANGLE)
            )
        );

        commandMap.put(
            "readyMid",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_MID),
                new Intake.IntakeSetAngleCommand(m_intake)
            )
        );

        commandMap.put(
            "readyTop",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_LAUNCH_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_LAUNCHING_ANGLE)
            )
        );

        commandMap.put(
            "readySubstation",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_HIGH_SUBSTATION_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_HIGH_SUBSTATION_ANGLE)
            )
        );

        commandMap.put(
            "place",
            new Intake.IntakeSetOutputCommand(m_intake)
        );

        commandMap.put(
            "intakeCube",
            new SequentialCommandGroup(
                new InstantCommand(() -> isCone = false),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CUBE_SPEED)
            )
        );

        commandMap.put(
            "intakeTiltedCone",
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = true),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
            )
        );
        
        commandMap.put(
            "intakeUprightCone",
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = false),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
            )
        );
        
        commandMap.put(
            "zero",
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        commandMap.put("waitQuarter", new WaitCommand(.25));
        commandMap.put("waitHalf", new WaitCommand(.5));
        commandMap.put("waitOne", new WaitCommand(1));

        commandMap.put(
            "autoPlaceConeTop",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_LAUNCH_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_LAUNCHING_ANGLE),
                new WaitCommand(1),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CUBE_SPEED),
                new WaitCommand(1),
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE),
                new WaitCommand(1)
            )
        );

        commandMap.put(
            "autoPlaceCubeTop",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_LAUNCH_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_LAUNCHING_ANGLE),
                new WaitCommand(1),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED),
                new WaitCommand(1),
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE),
                new WaitCommand(1)
            )
        );

        commandMap.put(
            "autoPickupCube",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PICKUP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOT_ANGLE),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CUBE_SPEED)
            )
        );

        commandMap.put(
            "autoPickupCone",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PICKUP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOT_ANGLE),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
            )
        );

        commandMap.put(
            "backwardsBalance",
            new SequentialCommandGroup(
                new BalanceCommand(m_swerve, false)
            )
        );
    }

    void configureArmAndIntake() {

        tiltUp.toggleOnTrue(new Intake.IntakeChangeTiltCommand(m_intake, 1));
        tiltDown.toggleOnTrue(new Arm.ArmChangeTiltCommand(m_arm, -1));
        cubeButton.toggleOnTrue( 
            new SequentialCommandGroup(
                new InstantCommand(() -> isCone = false),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CUBE_SPEED)
            )
        );
        cubeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        sidewaysConeButton.toggleOnTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = true),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
            )
        );
        sidewaysConeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        uprightConeButton.toggleOnTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = false),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
            )
        );
        uprightConeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        readyBotButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PICKUP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOT_ANGLE)
            )
        );

        readyMidButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_MID),
                new Intake.IntakeSetAngleCommand(m_intake)
            )
        );

        readyTopButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_LAUNCH_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_LAUNCHING_ANGLE)
            )
        );

        readySubstationButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_HIGH_SUBSTATION_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_HIGH_SUBSTATION_ANGLE)
            )
        );
       
        placeButton.toggleOnTrue(
            new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)
        );
        placeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand((m_intake), IntakeConstants.INTAKE_CUBE_SPEED),
                new WaitCommand(.5),
                new Intake.IntakeSetOutputCommand(m_intake, 0),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        Shuffleboard.getTab("Arm and Intake").addBoolean("Is Cone", () -> isCone);
        Shuffleboard.getTab("Arm and Intake").addBoolean("Is Bottom Cone", () -> isBottomCone);
        Shuffleboard.getTab("Arm and Intake").add("Intake", m_intake);
        Shuffleboard.getTab("Arm and Intake").add("Arm", m_arm);
    }

    void configureAuto() {
        autonChooser.setDefaultOption("Bottom: Place and Dock", autoBuilder.fullAuto(AutoConstants.BotPlaceAndDockAuto));
        autonChooser.addOption("Bot 2 Piece", autoBuilder.fullAuto(AutoConstants.Bot2Piece));
        autonChooser.addOption("Middle: Place and Dock", autoBuilder.fullAuto(AutoConstants.MidPlaceAndDockAuto));
        autonChooser.addOption("Top: Place And Dock", autoBuilder.fullAuto(AutoConstants.TopPlaceAndDockAuto));
        Shuffleboard.getTab("Auto").add("Auto Routes", autonChooser);
    }


    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public void teleopInit() {
        Command auton = autonChooser.getSelected();
        if (auton != null){
            auton.cancel();
        }

    }
}