package frc.robot;


import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;
import static frc.robot.Constants.RobotConstants.commandMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystem.swerve.command.BalanceCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand.ControlMode;


public class RobotContainer {
    /* Setting Joystick Buttons */
    private final Joystick driveStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);
    private final ControllerInfo info = new ControllerInfo();

    private final JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, JoystickConstants.LOCK_SWERVE_ROTATION);
    private final JoystickButton switchDriveModeRobotCentricButton = new JoystickButton(driveStick, JoystickConstants.SWITCH_DRIVE_MODE_ROBOT_CENTRIC);
    private final JoystickButton resetGyroButton = new JoystickButton(driveStick, JoystickConstants.RESET_GYRO);
    private final JoystickButton noForwardButton = new JoystickButton(driveStick, JoystickConstants.NO_FORWARD);
    private final JoystickButton balanceButton = new JoystickButton(driveStick, JoystickConstants.BALANCE);
    // private final JoystickButton limitSwerveSpeedButton = new JoystickButton(driveStick, JoystickConstants.LIMIT_SWERVE_SPEED);

    private final JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private final JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private final JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private final JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private final JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private final JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    private final JoystickButton sidewaysConeButton = new JoystickButton(controlStick, JoystickConstants.SIDEWAYS_CONE);
    private final JoystickButton readySubstationButton = new JoystickButton(controlStick, JoystickConstants.SUBSTATION_PICKUP);

    private final DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private TriModeSwerveCommand swerveCommand;
    private BalanceCommand balanceCommand;
    public static boolean isCone; // Changes with coneButton/cubeButton
    public static boolean isBottomCone = false; // Changes with Orientation buttons

    private final PathFollowingSwerve m_swerve = HardwareSwerveFactory.makeSwerve();
    private final Arm m_arm = makeArm();
    private final Intake m_intake = makeIntake();
    // private final Vision m_vision = HardwareVisionFactory.makeVision();

    /**Both PID constants need to be tested */
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_swerve::getCurrentPose, 
        m_swerve::resetPose, 
        new PIDConstants(5, 0, 0), 
        new PIDConstants(.5, 0, 0), 
        m_swerve::moveRobotCentric, 
        commandMap, 
        m_swerve);    
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
        balanceCommand = new BalanceCommand(m_swerve, true); // TODO: Change based on Auto.
        m_swerve.setDefaultCommand(swerveCommand);

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentricButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;}));
        switchDriveModeRobotCentricButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        /* Slow Side to Side movement, RobotCentric */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        // limitSwerveSpeedButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        // limitSwerveSpeedButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        /* Balance Robot, should end on balancing 10 cycles */
        balanceButton.toggleOnTrue(
            new InstantCommand(
                () -> m_swerve.setDefaultCommand(balanceCommand)
            )
        );
        balanceButton.toggleOnFalse(
            new InstantCommand(
                () -> m_swerve.setDefaultCommand(swerveCommand)
            )
        );

        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }

    void configureCommands() {
        commandMap.put(
            "readyBot", 
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_GROUND_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_BOT),
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
                new Intake.IntakeSetAngleCommand(m_intake)
            )
        );

        commandMap.put(
            "readySubstation",
            new SequentialCommandGroup(
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_SUBSTATION_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_PLACE_TOP),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_TRAY_PICKUP_ANGLE)
            )
        );

        commandMap.put(
            "place",
            new Intake.IntakeSetOutputCommand(m_intake, true)
        );

        commandMap.put(
            "intakeCube",
            new SequentialCommandGroup(
                new InstantCommand(() -> isCone = false),
                new Intake.IntakeSetOutputCommand(m_intake, false)
            )
        );

        commandMap.put(
            "intakeTiltedCone",
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = true),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, false)
            )
        );
        
        commandMap.put(
            "intakeUprightCone",
            new SequentialCommandGroup(
                new InstantCommand(() -> isBottomCone = false),
                new InstantCommand(() -> isCone = true),
                new Intake.IntakeSetOutputCommand(m_intake, false)
            )
        );
        
        commandMap.put(
            "zero",
            new SequentialCommandGroup(
                new Intake.IntakeSetOutputCommand(m_intake),
                new Intake.IntakeSetAngleCommand(m_intake, IntakeConstants.INTAKE_ZERO_ANGLE),
                new Arm.ArmSetWinchOutputCommand(m_arm, ArmConstants.ARM_RETRACT),
                new Arm.ArmSetTiltAngleCommand(m_arm, ArmConstants.ARM_ZERO_ANGLE)
            )
        );

        commandMap.put("waitQuarter", new WaitCommand(.25));
        commandMap.put("waitHalf", new WaitCommand(.5));
        commandMap.put("waitOne", new WaitCommand(1));
    }

    void configureArmAndIntake() {

        cubeButton.toggleOnTrue(commandMap.get("intakeCube"));
        cubeButton.toggleOnFalse(commandMap.get("zero"));

        sidewaysConeButton.toggleOnTrue(commandMap.get("intakeTiltedCone"));
        sidewaysConeButton.toggleOnFalse(commandMap.get("zero"));

        uprightConeButton.toggleOnTrue(commandMap.get("intakeUprightCone"));
        uprightConeButton.toggleOnFalse(commandMap.get("zero"));

        readyBotButton.toggleOnTrue(commandMap.get("readyBot"));

        readyMidButton.toggleOnTrue(commandMap.get("readyMid"));

        readyTopButton.toggleOnTrue(commandMap.get("readyTop"));

        readySubstationButton.toggleOnTrue(commandMap.get("readySubstation"));
       
        placeButton.toggleOnTrue(commandMap.get("place"));
        placeButton.toggleOnFalse(commandMap.get("zero"));

        Shuffleboard.getTab("Arm and Intake").addBoolean("Is Cone", () -> isCone);
        Shuffleboard.getTab("Arm and Intake").addBoolean("Is Bottom Cone", () -> isBottomCone);
        Shuffleboard.getTab("Arm and Intake").add("Intake", m_intake);
        Shuffleboard.getTab("Arm and Intake").add("Arm", m_arm);
    }

    void configureAuto() {
        autonChooser.setDefaultOption("Bottom: Place and Move", autoBuilder.fullAuto(AutoConstants.BotPlaceAndMoveAuto));
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
