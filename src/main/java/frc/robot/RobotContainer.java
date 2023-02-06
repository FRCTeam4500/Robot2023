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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Arm.Position;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;

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
    private SwerveCommand swerveCommand;
    private boolean isCone; // Changes with coneButton/cubeButton
    private boolean isBottomConeOrientation; // Changes with Orientation buttons
    

    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

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
        coneButton.toggleOnTrue(new InstantCommand(() -> {isCone = true;})); //0.55 is the past intake speed
        coneButton.toggleOnFalse(new InstantCommand());

        cubeButton.toggleOnTrue(new InstantCommand(() -> {isCone = false;}));
        cubeButton.toggleOnFalse(new InstantCommand());

        uprightConeButton.toggleOnTrue(new InstantCommand(() -> {isBottomConeOrientation = false;}));
        uprightConeButton.toggleOnFalse(new InstantCommand());

        sidewaysConeButton.toggleOnTrue(new InstantCommand(() -> {isBottomConeOrientation = true;}));
        sidewaysConeButton.toggleOnFalse(new InstantCommand());

        placeButton.toggleOnTrue(
                new Intake.IntakeSetOutputCommand(
                        m_intake, isCone ? IntakeConstants.INTAKE_CUBE_SPEED : IntakeConstants.INTAKE_CONE_SPEED
                )
        );
        placeButton.toggleOnFalse(
                new Intake.IntakeSetOutputCommand(
                        m_intake, 0
                )
        );

        /*  */
        readyTopButton.toggleOnTrue(new ParallelCommandGroup(
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_TRAY_PICKUP)
        )));
        readyTopButton.toggleOnFalse(new InstantCommand());

        readyMidButton.toggleOnTrue(new ParallelCommandGroup(
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE)
                        .andThen(new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_MID)),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_TRAY_ANGLE)
        ));
        readyMidButton.toggleOnFalse(new InstantCommand());

        readyBotButton.toggleOnTrue(new ParallelCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_BOT)
                        .andThen(new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PICKUP_ANGLE)),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_BOTTOM_ANGLE)
        ));
        readyBotButton.toggleOnFalse(new InstantCommand());

        retractButton.toggleOnTrue(new ParallelCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
                        .andThen(new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_RETRACT)),
                new Intake.IntakeSetOutputCommand(m_intake, IntakeConstants.INTAKE_RETRACTED_ANGLE)
        ));
        retractButton.toggleOnFalse(new InstantCommand());
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