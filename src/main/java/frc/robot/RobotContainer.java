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

public class RobotContainer {
    /* Setting Joystick Buttons */
    private Joystick driveStick = new Joystick(0);
    private Joystick controlStick = new Joystick(1);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, 1);
    private JoystickButton switchDriveModeRobotCentric = new JoystickButton(driveStick, 4);
    private JoystickButton alignSwerveToAngle = new JoystickButton(driveStick, 8);
    private JoystickButton alignSwerveReverse = new JoystickButton(driveStick, 7);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 10);
    private JoystickButton limitSwerveSpeed = new JoystickButton(driveStick, 2);
    private JoystickButton noForwardButton = new JoystickButton(driveStick, 9);
    //private JoystickButton funButton = new JoystickButton(driveStick, #);

    private JoystickButton coneButton = new JoystickButton(controlStick, 11);
    private JoystickButton cubeButton = new JoystickButton(controlStick, 12);

    // change buttonNumbers later
    private JoystickButton placerGroundButton = new JoystickButton(controlStick, 4);
    private JoystickButton placerBottomButton = new JoystickButton(controlStick, 1);
    private JoystickButton placerMiddleButton = new JoystickButton(controlStick, 2);
    private JoystickButton placerTopButton = new JoystickButton(controlStick, 3);
    private JoystickButton placerRetractedButton = new JoystickButton(controlStick, 4);
    
    private JoystickButton intakeButton = new JoystickButton(controlStick, 5);
    private JoystickButton bottomConeOrientation = new JoystickButton(controlStick, 5);
    private JoystickButton topConeOrientation = new JoystickButton(controlStick, 6);
    
    
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

        // lambdas to define what happens for each joystick button for configuring swerve
        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentric.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;}));
        switchDriveModeRobotCentric.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        /* Slow Side to Side movement */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        alignSwerveToAngle.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        alignSwerveToAngle.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        alignSwerveReverse.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        alignSwerveReverse.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        limitSwerveSpeed.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        limitSwerveSpeed.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        resetGyro.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));



        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

        Shuffleboard.getTab("Swerve").add("swervefield", m_swerve).withWidget("Field");
    }

    void configureArmAndIntake() {
        // Method where we will define lambdas for joystick buttons to control Arm and Intake
        coneButton.toggleOnTrue(new InstantCommand(() -> {m_intake}));
        coneButton.toggleOnFalse();

        // placerBottomButton.toggleOnTrue(new ParallelCommandGroup(
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