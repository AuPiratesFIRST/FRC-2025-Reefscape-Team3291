// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.s
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto.nothing;
//import frc.robot.subsystems.VisionSubsystem;

// import frc.robot.commands.ColorChangingCMD;
// import frc.robot.commands.SwerveDrive;
// import frc.robot.commands.Auto.MN_MildAuto;
// import frc.robot.subsystems.PreferencesSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ColorChanger;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    //controllers
    public CommandJoystick controller0 = new CommandJoystick(0);
    public CommandJoystick controller1 = new CommandJoystick(1);

    //buttons
    public final JoystickButton backToggleButton = new JoystickButton(controller0.getHID(), Constants.buttonList.back);
    public final JoystickButton aToggleButton = new JoystickButton(controller1.getHID(), Constants.buttonList.a);
    public final JoystickButton colorToggleButton = new JoystickButton(controller1.getHID(), Constants.buttonList.l3);
    public final JoystickButton robotCentricButton = new JoystickButton(controller0.getHID(), Constants.buttonList.r3);

    //subsystems
    //public PreferencesSubsystem preferencesSubsystem = new PreferencesSubsystem();
    //public VisionSubsystem visionSubsystem = new VisionSubsystem();
    //public ColorChanger colorChanger = new ColorChanger();
    //private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //commands
    //Vision
    //public DriveToApriltag driveToApriltag = new DriveToApriltag(swerveSubsystem, visionSubsystem);
    //public AprilTagTest aprilTagTest = new AprilTagTest(visionSubsystem);
    //subsystems\\
    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // Subsystem initialization

        // Register Named Commands
        //NamedCommands.registerCommand("ColorChangingCMD", new ColorChangingCMD(colorChanger));
        //NamedCommands.registerCommand("DriveToApriltag", new DriveToApriltag(swerveSubsystem, visionSubsystem));
        configureBindings();

        //Controller 0
        //Controller1
        //controller1.button(Constants.buttonList.a).onTrue(aprilTagTest);
        //Autonomous
        // autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("AutoChooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //swerveSubsystem.setDefaultCommand(
        //  new SwerveDrive(
        //    swerveSubsystem,
        //    visionSubsystem,
        //    () -> controller0.getRawAxis(1),
        //   () -> -controller0.getRawAxis(0),
        //    () -> controller0.getRawAxis(4),
        //    () -> robotCentricButton.getAsBoolean(),
        //    () -> backToggleButton.getAsBoolean()
        //  )
        //);

    }

    public Command getAutonomousCommand() {
        // TODO Auto-generated method stub

        //return new PathPlannerAuto("Launch Auto");
        return new nothing();
        //return new MildAuto(swerveSubsystem);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
}
