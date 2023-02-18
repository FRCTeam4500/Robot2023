package frc.robot;


import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Arm.Position;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;

import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
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
    private final JoystickButton balanceButton = new JoystickButton(driveStick, JoystickConstants.BALANCE);

    private final JoystickButton coneButton = new JoystickButton(controlStick, JoystickConstants.CONE_INTAKE);
    private final JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private final JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private final JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private final JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private final JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private final JoystickButton retractButton = new JoystickButton(controlStick, JoystickConstants.RETRACT);
    private final JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    private final JoystickButton sidewaysConeButton = new JoystickButton(controlStick, JoystickConstants.SIDEWAYS_CONE);

    private final DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private TriModeSwerveCommand swerveCommand;
    private boolean isCone; // Changes with coneButton/cubeButton
    private boolean isBottomCone; // Changes with Orientation buttons

    
    private static final HashMap<String, Command> COMMAND_MAP = new HashMap<>();
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    private final PathFollowingSwerve m_swerve = HardwareSwerveFactory.makeSwerve();
     private final Arm m_arm = makeArm();
     private final Intake m_intake = makeIntake();

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

    void configureCommands() {
        // Define commonly used commands
        // They can then be called using COMMAND_MAP.get("key");
        COMMAND_MAP.put("Ready Bot", new SequentialCommandGroup(
            new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
            new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_BOT, isBottomCone),
            new Intake.IntakeSetAngleCommand(m_intake,isCone, isBottomCone)
            ));

        COMMAND_MAP.put("Ready Mid", new SequentialCommandGroup(
            new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
            new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_MID, isBottomCone),
            new Intake.IntakeSetAngleCommand(m_intake,isCone, isBottomCone)
            ));

        COMMAND_MAP.put("Ready Top", new SequentialCommandGroup(
            new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
            new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP, isBottomCone),
            new Intake.IntakeSetAngleCommand(m_intake,isCone, isBottomCone)
            ));

        COMMAND_MAP.put("Place", new Intake.IntakeSetOutputCommand(m_intake, isCone));
    }
    void configureSwerve() {
        swerveCommand = new TriModeSwerveCommand(m_swerve, driveStick, info, messages);
        m_swerve.setDefaultCommand(swerveCommand);

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentricButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;}));
        switchDriveModeRobotCentricButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        /* Slow Side to Side movement */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        alignSwerveToAngleButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        alignSwerveToAngleButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        alignSwerveReverseButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        alignSwerveReverseButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        limitSwerveSpeedButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        limitSwerveSpeedButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        balanceButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.balance = true;}));
        balanceButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.balance = false; swerveCommand.controlMode = ControlMode.FieldCentric;}));

        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }

    void configureArmAndIntake() {
        coneButton.toggleOnTrue(new InstantCommand(() -> {isCone = true;}));
        cubeButton.toggleOnTrue(new InstantCommand(() -> {isCone = false;}));
        uprightConeButton.toggleOnTrue(new InstantCommand(() -> {isBottomCone = false;}));
        sidewaysConeButton.toggleOnTrue(new InstantCommand(() -> {isBottomCone = true;}));

        readyTopButton.toggleOnTrue(COMMAND_MAP.get("Ready Top"));



    }

    void configureAuto() {
        
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