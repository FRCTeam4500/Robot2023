package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.dashboard.DashboardNumberDisplay;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand.ControlMode;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ControllerInfo;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Vision;
import frc.robot.subsystem.Arm.ArmSetTiltAngleCommand;

public class RobotContainer {

    //Initialize subsystems
    private PathFollowingSwerve swerve = HardwareSwerveFactory.makeSwerve();
    private Vision vision = Vision.makeVision();
    private Arm arm = new Arm();
    private Intake intake = new Intake();


    //Initialize Joysticks and Buttons
    private Joystick driveStick = new Joystick(0);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, 1);
    private JoystickButton switchDriveModeRobotCentric = new JoystickButton(driveStick, 4);
    private JoystickButton alignSwerveToAngle = new JoystickButton(driveStick, 8);
    private JoystickButton alignSwerveReverse = new JoystickButton(driveStick, 7);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 10);
    private JoystickButton limitSwerveSpeed = new JoystickButton(driveStick, 2);
    private JoystickButton noForwardButton = new JoystickButton(driveStick, 9);

    private Joystick controlStick = new Joystick(1);
    
    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    private TriModeSwerveCommand swerveCommand;

    boolean isPlacing = false;
    boolean isIntaking = false;

    public RobotContainer(){
        configureControls();
        configureSwerve();
        configureVision();
        // configureAutonomous();
    }

    void configureControls(){
        info.xSensitivity = 4;
        info.ySensitivity = 4;
        info.zSensitivity = 3.5;
        info.xDeadzone = 0.2;
        info.yDeadzone = 0.2;
        info.zDeadzone = 0.2;
        Shuffleboard.getTab("Driver Controls").add("Driver Controls", info);
        Shuffleboard.getTab("Driver Controls").add("Messages", messages);
        //Shuffleboard.getTab("Driver Controls").add("Intake Camera", camera);
    }

    void configureSwerve() {
        swerveCommand = new TriModeSwerveCommand(swerve, driveStick, info, messages);
        swerveCommand.controlMode = ControlMode.FieldCentric;

        //switchDriveModeRobotCentric.toggleOnTrue(() -> swerveCommand.controlMode = ControlMode.RobotCentric; new InstantCommand(turretLights.setCurrentRoutine(Lights.Routines.blueorangereverse);));
        //switchDriveModeRobotCentric.toggleOnFalse(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; resetLights();});

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        alignSwerveToAngle.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        alignSwerveToAngle.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        alignSwerveReverse.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        alignSwerveReverse.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        limitSwerveSpeed.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        limitSwerveSpeed.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        resetGyro.toggleOnTrue(new InstantCommand(() -> {swerve.resetRobotAngle();}));

        // no lights subsystem D:
        swerve.setDefaultCommand(swerveCommand);
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

    }

    void configureOuttake() {
        Shuffleboard.getTab("Intake").add("Intake", intake);
        Shuffleboard.getTab("Intake").add("Arm", arm);
    }

    void configureVision() {
        Shuffleboard.getTab("Driving").add("Distance", new DashboardNumberDisplay("Distance", () -> Vision.VisionDistanceCalculator.calculateDistance(vision)));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
    

    public void teleopInit() {
        Command auton = autonChooser.getSelected();
        if (auton != null){
            auton.cancel();
        }
        new ArmSetTiltAngleCommand(arm, Constants.ArmConstants.ARM_ZERO_ANGLE).schedule(); //deploy the arm
    }
}
