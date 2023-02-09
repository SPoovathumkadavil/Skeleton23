package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.utility.ControllerInfo;
import frc.robot.subsystem.DriveTrain.PathFollowingSwerve;
import frc.robot.subsystem.DriveTrain.SwerveCommand;
import static frc.robot.subsystem.DriveTrain.makeSwerve;

public class RobotContainer {

    private final Joystick driveStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);
    private final ControllerInfo info = new ControllerInfo();

    /* Drive Joystick Buttons */
    private final JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, Constants.JoystickConstants.LOCK_SWERVE_ROTATION);
    private final JoystickButton switchDriveModeRobotCentricButton = new JoystickButton(driveStick, Constants.JoystickConstants.SWITCH_DRIVE_MODE_ROBOT_CENTRIC);
    private final JoystickButton alignSwerveToAngleButton = new JoystickButton(driveStick, Constants.JoystickConstants.ALIGN_SWERVE_TO_ANGLE);
    private final JoystickButton alignSwerveReverseButton = new JoystickButton(driveStick, Constants.JoystickConstants.ALIGN_SWERVE_REVERSE);
    private final JoystickButton resetGyroButton = new JoystickButton(driveStick, Constants.JoystickConstants.RESET_GYRO);
    private final JoystickButton limitSwerveSpeedButton = new JoystickButton(driveStick, Constants.JoystickConstants.LIMIT_SWERVE_SPEED);
    private final JoystickButton noForwardButton = new JoystickButton(driveStick, Constants.JoystickConstants.NO_FORWARD);
    private final JoystickButton balanceButton = new JoystickButton(driveStick, Constants.JoystickConstants.BALANCE);

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();
    private final DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);

    private final PathFollowingSwerve swerve = makeSwerve();

    private SwerveCommand swerveCommand;


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
        swerveCommand = new SwerveCommand(swerve, driveStick, info, messages);

        swerve.setDefaultCommand(swerveCommand);

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        switchDriveModeRobotCentricButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.RobotCentric;}));
        switchDriveModeRobotCentricButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.FieldCentric;}));

        /* Slow Side to Side movement */
        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        alignSwerveToAngleButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        alignSwerveToAngleButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.FieldCentric;}));

        alignSwerveReverseButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        alignSwerveReverseButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = SwerveCommand.ControlMode.FieldCentric;}));

        limitSwerveSpeedButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        limitSwerveSpeedButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        resetGyroButton.toggleOnTrue(new InstantCommand(swerve::resetRobotAngle));

        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
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
