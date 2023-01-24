package frc.robot;

import frc.robot.subsystem.Swerve.OdometricSwerve;
import frc.robot.subsystem.Swerve.SwerveCommand;
import frc.robot.utility.ControllerInfo;

import static frc.robot.subsystem.Swerve.makeSwerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class RobotContainer {
    private final OdometricSwerve m_swerve = makeSwerve();

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

    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);
    private SwerveCommand swerveCommand;

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureControls();
        configureSwerve();
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

        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
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