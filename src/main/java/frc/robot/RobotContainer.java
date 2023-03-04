package frc.robot;


import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;

import java.util.HashMap;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystem.Arm.Position;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autonomous.routines.BasicAuto;
import frc.robot.autonomous.routines.TestingAuto;
import frc.robot.autonomous.routines.Top2PieceAuto;
import frc.robot.component.hardware.SparkMaxComponent;
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

    private final JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private final JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private final JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private final JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private final JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private final JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    private final JoystickButton sidewaysConeButton = new JoystickButton(controlStick, JoystickConstants.SIDEWAYS_CONE);
    private final JoystickButton readySubstationButton = new JoystickButton(controlStick, JoystickConstants.SUBSTATION_PICKUP);

    private final JoystickButton goInButton = new JoystickButton(controlStick, JoystickConstants.GO_IN);
    private final JoystickButton goOutButton = new JoystickButton(controlStick, JoystickConstants.GO_OUT);
    private final JoystickButton zeroIntakeButton = new JoystickButton(controlStick, 11);

    private final DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private TriModeSwerveCommand swerveCommand;
    public static boolean isCone; // Changes with coneButton/cubeButton
    public static boolean isBottomCone = true; // Changes with Orientation buttons
    
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();
    private final PathFollowingSwerve m_swerve = HardwareSwerveFactory.makeSwerve();
    private final Arm m_arm = makeArm();
    private final Intake m_intake = makeIntake();
    private final Vision m_vision = HardwareVisionFactory.makeVision();

    public RobotContainer() {
        configureControls();
        // configureCommands();
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
        m_swerve.setDefaultCommand(swerveCommand);

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentricButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;}));
        switchDriveModeRobotCentricButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        /* Slow Side to Side movement */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        // alignSwerveToAngleButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        // alignSwerveToAngleButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        // alignSwerveReverseButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        // alignSwerveReverseButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        // limitSwerveSpeedButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        // limitSwerveSpeedButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }

    void configureArmAndIntake() {

        cubeButton.toggleOnTrue( // Intakes cones
            new SequentialCommandGroup(
                new InstantCommand(() -> isCone = false),
                new Intake.IntakeSetOutputCommand(m_intake, false, false)
            )
        );
        cubeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, true, false),
                new Intake.IntakeSetAngleCommand(m_intake, false, false, true, false),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        sidewaysConeButton.toggleOnTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = true),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, false, false)
            )
        );
        sidewaysConeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, true, false),
                new Intake.IntakeSetAngleCommand(m_intake, false, false, true, false),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        uprightConeButton.toggleOnTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = false),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, false, false)
            )
        );
        uprightConeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, true, false),
                new Intake.IntakeSetAngleCommand(m_intake, false, false, true, false),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        readyBotButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_BOT),
                new Intake.IntakeSetAngleCommand(m_intake, false, false, false, false)
            )
        );

        readyMidButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_PLACE_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_MID),
                new Intake.IntakeSetAngleCommand(m_intake, true, false, false, false)
            )
        );

        readyTopButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_LAUNCH_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, true, true, false, false)
            )
        );

        readySubstationButton.toggleOnTrue(
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_SUBSTATION_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, true, false, false, true)
            )
        );
       
        placeButton.toggleOnTrue(
            new Intake.IntakeSetOutputCommand(m_intake, false, true)
        );
        placeButton.toggleOnFalse(
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake, true, false),
                new Intake.IntakeSetAngleCommand(m_intake, false, false, true, false),
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
       autonChooser.setDefaultOption("Basic", new BasicAuto(m_swerve, m_arm, m_intake));
       autonChooser.addOption("Top2Piece", new Top2PieceAuto(m_swerve, m_arm));
       autonChooser.addOption("Testing", new TestingAuto(m_swerve));
       Shuffleboard.getTab("Driver Controls").add("Auto Route", autonChooser);
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