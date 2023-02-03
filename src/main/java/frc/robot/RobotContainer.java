package frc.robot;

import frc.robot.subsystem.Swerve.OdometricSwerve;
import frc.robot.subsystem.Swerve.SwerveCommand;
import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Swerve.makeSwerve;
import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Swerve.SwerveCommand.ControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.Arm.Position;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;

public class RobotContainer {
    /* Setting Joystick Buttons */
    private Joystick driveStick = new Joystick(0);
    private Joystick controlStick = new Joystick(1);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, JoystickConstants.LOCK_SWERVE_ROTATION);
    private JoystickButton switchDriveModeRobotCentricButton = new JoystickButton(driveStick, JoystickConstants.SWITCH_DRIVE_MODE_ROBOT_CENTRIC);
    private JoystickButton alignSwerveToAngleButton = new JoystickButton(driveStick, JoystickConstants.ALIGN_SWERVE_TO_ANGLE);
    private JoystickButton alignSwerveReverseButton = new JoystickButton(driveStick, JoystickConstants.ALIGN_SWERVE_REVERSE);
    private JoystickButton resetGyroButton = new JoystickButton(driveStick, JoystickConstants.RESET_GYRO);
    private JoystickButton limitSwerveSpeedButton = new JoystickButton(driveStick, JoystickConstants.LIMIT_SWERVE_SPEED);
    private JoystickButton noForwardButton = new JoystickButton(driveStick, JoystickConstants.NO_FORWARD);
    //private JoystickButton funButton = new JoystickButton(driveStick, #);

    private JoystickButton coneButton = new JoystickButton(controlStick, JoystickConstants.CONE_INTAKE);
    private JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private JoystickButton readyGroundButton = new JoystickButton(controlStick, JoystickConstants.READY_GROUND);
    private JoystickButton retractButton = new JoystickButton(controlStick, JoystickConstants.RETRACT);
    private JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    private JoystickButton sidewaysConeButton = new JoystickButton(controlStick, JoystickConstants.SIDEWAYS_CONE);

    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private SwerveCommand swerveCommand;
    private boolean isCone;
    private boolean isBottomConeOrientation;
    private Position position;
    

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    private final OdometricSwerve m_swerve = makeSwerve();
    private final Arm m_arm = makeArm();
    private final Intake m_intake = makeIntake();

    public RobotContainer() {
        configureControls();
        configureSwerve();
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
        swerveCommand = new SwerveCommand(m_swerve, driveStick, info);
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

        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

        Shuffleboard.getTab("Swerve").add("swervefield", m_swerve).withWidget("Field");
    }

    void configureArmAndIntake() {
        // coneButton.toggleOnTrue();
        // coneButton.toggleOnFalse();

        // armBottomButton.toggleOnTrue(new ParallelCommandGroup(
        //     new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE)
        //         .andThen(new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_BOT)),
        //     new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOTTOM_CONE_PLACE)
        // ));
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