// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.Rotation;

//import frc.robot.subsystems.HangSubsystem;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive;;
    private final CoralSubsystem m_coralSubSystem;
    private final AlgaeSubsystem m_algaeSubsystem;
    //private final HangSubsystem m_hangSubsystem = new HangSubsystem();

    // The driver's controller
    private CommandXboxController m_driverController;
    private CommandXboxController m_operatorController;

    // A chooser for autonomous commands
    private SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        m_robotDrive = new DriveSubsystem();
        m_coralSubSystem = new CoralSubsystem();
        m_algaeSubsystem = new AlgaeSubsystem();

        m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand( () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

        // Set the ball intake to in/out when not running based on internal state
        m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

        // Set the hang arm to hold when not running
        //m_hangSubsystem.setDefaultCommand(m_hangSubsystem.idleCommand());

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("ElevatorLevel2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
        NamedCommands.registerCommand("ScoreCoral", m_coralSubSystem.runIntakeCommand().withTimeout(1));
        NamedCommands.registerCommand("IntakeCoral", m_coralSubSystem.reverseIntakeCommand().withTimeout(1));
        NamedCommands.registerCommand("ElevatorFeederStation", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));

        // Add commands to the autonomous command chooser
        // m_chooser.setDefaultOption("Leave Auto", leaveAutoCommand(config, thetaController));
        // m_chooser.addOption("Middle Auto", middleAutoCommand(config, thetaController));
        // m_chooser.addOption("Right Auto (No human player)", rightAutoCommand(config, thetaController, false));
        // m_chooser.addOption("Left Auto", leftAutoCommand(config, thetaController));

        // Build an auto chooser. This will use Commands.none() as the default option.
        m_chooser = AutoBuilder.buildAutoChooser();

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // Create ProfiledPIDController for theta controller
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_chooser.addOption("Taxi Out", leaveAutoCommand(config, thetaController));
        
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    /* 
    *  =========================================================
    *                   COMMAND BINDINGS
    *  =========================================================
    * 
    */

    /**
    * Use this method to define your button->command mappings. Buttons can be created by
    * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
    * {@link JoystickButton}.
    */
    private void configureButtonBindings() {
        
        // Left Stick Button -> Set swerve to X
        m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
        
        // Start Button -> Zero swerve heading
        m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

        // Right Trigger -> Run ball intake, set to leave out when idle
        m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold)
            .whileTrue(m_algaeSubsystem.runIntakeCommand());
        // Left Trigger -> Run ball intake in reverse, set to stow when idle
        m_driverController.leftTrigger(OIConstants.kTriggerButtonThreshold)
            .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

        // Left Bumper -> Run hang arm, set to hold when idle
        //m_driverController.leftBumper().whileTrue(m_hangSubsystem.runHangCommand());
        // Right Bumper -> Run hang arm in reverse, set to release when idle
        //m_driverController.rightBumper().whileTrue(m_hangSubsystem.reverseHangCommand());



        // Left Bumper -> Run coral tube intake
        m_operatorController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());
        // Left Trigger -> Run coral tube intake slower
        // m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold)
        // .whileTrue(m_coralSubSystem.runIntakeSlowCommand());
        // Right Bumper -> Run coral tube intake in reverse
        m_operatorController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

        // A Button -> Elevator/Arm to human player position, set ball intake to stow when idle
        m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation)
            .alongWith(m_algaeSubsystem.stowCommand()));
        // B Button -> Elevator/Arm to level 1 position
        //m_operatorController.b().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
        // X Button -> Elevator/Arm to level 2 position
        m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
        // Y Button -> Elevator/Arm to level 3 position
        m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
        // Y Button -> Elevator/Arm to level 4 position
        // m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

        /*
         *  Elevator & Arm manual run commands
         */
        // // Left Trigger -> Raise Elevator
        // m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_coralSubSystem.runElevatorCommand());
        // // Right Trigger -> Lower Elevator
        // m_operatorController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_coralSubSystem.reverseElevatorCommand());
        // // Left Bumper -> Rotate arm forward
        // m_operatorController.leftBumper().whileTrue(m_coralSubSystem.runArmCommand());
        // // Right Bumper -> Rotate arm backward
        // m_operatorController.rightBumper().whileTrue(m_coralSubSystem.reverseArmCommand());

    }

    public double getSimulationTotalCurrentDraw() {
        // for each subsystem with simulation
        return m_coralSubSystem.getSimulationCurrentDraw()
            + m_algaeSubsystem.getSimulationCurrentDraw();
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // Create ProfiledPIDController for theta controller
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        // Send selected auto command to Robot.java
        //return m_chooser.getSelected();
        // rightAutoCommand(config, thetaController, false);
        // leftAutoCommand(config, thetaController);
        // middleAutoCommand(config, thetaController);
        // leaveAutoCommand(config, thetaController);
        // return leaveAutoCommand(config, thetaController);
        return m_chooser.getSelected();
    }
    /* 
     *  =========================================================
     *                   AUTONOMOUS ROUTINES
     *  =========================================================
     *
     *                            +X
     *                          ^ 
     *                          |
     *                   +Y <---O---> -Y
     *                          |  
     *                          v 
     *                       -X
     * 
     *                      Forward = +X
     *           Left/CCW = +Y        Right/CW = -Y
     *                     Backward = -X
     *          
     *          
     *  =========================================================
     * 
     *  1. Leave Auto Routine (3 (Leave) = 3 pts)
     *  2. Middle Auto Routine (6 (Coral) + 3 (Leave) = 9 pts)
     *  3. Right Auto Routine (6 (Coral) + 3 (Leave) = 9 pts)
     *  4. Left Auto Routine (6 (Coral) + 3 (Leave) = 9 pts)
     * 
     *  =========================================================
     */
    
    /* Auto Routine for soley leave points
     * 
     * Starting position: 
     *  - Make sure the algae intake is facing TOWARDS YOU 
     *  - RSL light should be facing AWAY from you
     * 
     * 
    */
    public Command leaveAutoCommand(TrajectoryConfig config, ProfiledPIDController thetaController) {
        
        // Drive backward command
        Trajectory backwardTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior Waypoint
            List.of(new Translation2d(1, 0)),
            // End 2 meters straight behind of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

        SwerveControllerCommand backwardCommand = new SwerveControllerCommand(
            backwardTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(backwardTrajectory.getInitialPose());

        Command correctGyroCommand = m_robotDrive.setAngleOffsetCommand(180);

        return backwardCommand.andThen(correctGyroCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)));
    }
    
    // // Auto Routine for middle of field
    // public Command middleAutoCommand(TrajectoryConfig config, ProfiledPIDController thetaController) {

    //     // Drive forward command
    //     Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Interior Waypoint
    //         List.of(new Translation2d(1, 0)),
    //         // End 2.15 meters straight ahead of where we started, facing forward
    //         new Pose2d(2.15, 0, new Rotation2d(0)),
    //         config);

    //     SwerveControllerCommand forwardCommand = new SwerveControllerCommand(
    //         forwardTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);

    //     // Lift algae out command (Go to level 3)
    //     Command liftAlgaeCommand = m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3);

    //     // Shimmy right command
    //     Trajectory shimmyRightTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Interior Waypoint
    //         List.of(new Translation2d(0, -0.1)),
    //         // End 0.17 meters right of where we started, facing forward
    //         new Pose2d(0, -0.17, new Rotation2d(0)),
    //         config);

    //     SwerveControllerCommand shimmyRightCommand = new SwerveControllerCommand(
    //         shimmyRightTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);

    //     // Score coral command
    //     Command scoreCoralCommand = m_coralSubSystem.reverseIntakeCommand().withTimeout(1);

    //     // Reset odometry to the starting pose of the trajectory.
    //     m_robotDrive.resetOdometry(forwardTrajectory.getInitialPose());

    //     // Go forward 88 inches -> Lift algae out of reef -> Shimmy to the right to be in line with branch -> Score coral -> Stop
    //     return forwardCommand.andThen(liftAlgaeCommand.andThen(shimmyRightCommand.andThen(scoreCoralCommand.andThen(
    //         () -> m_robotDrive.drive(0, 0, 0, false)))));
    // }

    // // Auto Routine for right side of field
    // public Command rightAutoCommand(TrajectoryConfig config, ProfiledPIDController thetaController, boolean humanPlayer) {
        
    //     // Go to correct position on reef to score coral
    //     Trajectory reefTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         List.of(new Pose2d(0, 0, new Rotation2d(0)), 
    //         // Move forward 137.25" and left 21.625"
    //         new Pose2d(Units.inchesToMeters(137.25), Units.inchesToMeters(21.625), new Rotation2d(0)),
    //         // Move left 100" and turn 150 degrees CCW (5PI/6 radians)
    //         new Pose2d(Units.inchesToMeters(137.25), Units.inchesToMeters(121.625), new Rotation2d(5 * Math.PI / 6))),
    //         config);
        
    //     SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
    //         reefTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,
    
    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);

    //     // Lift algae out command (Go to level 3)
    //     Command liftAlgaeCommand = m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3);

    //     // Shimmy right command
    //     Trajectory shimmyRightTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Interior Waypoint
    //         List.of(new Translation2d(0, -0.1)),
    //         // End 0.17 meters right of where we started, facing forward
    //         new Pose2d(0, -0.17, new Rotation2d(0)),
    //         config);

    //     SwerveControllerCommand shimmyRightCommand = new SwerveControllerCommand(
    //         shimmyRightTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);
        
    //     // Score coral command
    //     Command scoreCoralCommand = m_coralSubSystem.reverseIntakeCommand().withTimeout(1);

    //     // Reset odometry to the starting pose of the trajectory.
    //     m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    //     // Go to reef -> Lift algae out of reef -> Shimmy to the right to be in line with branch -> Score coral -> Stop
    //     return moveToReefCommand.andThen(liftAlgaeCommand.andThen(shimmyRightCommand.andThen(scoreCoralCommand.andThen(
    //         () -> m_robotDrive.drive(0, 0, 0, false)))));
        

        // /*
        //  * 
        //  *          HUMAN PLAYER CODE
        //  * 
        //  */
        // // Lower elevator to feeder station level command
        // Command lowerElevatorCommand = m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation);

        // // Go to coral station command
        // Trajectory coralStationTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the position of the reef
        //     new Pose2d(Units.inchesToMeters(137.25), Units.inchesToMeters(121.625), new Rotation2d(5 * Math.PI / 6)),
        //     // Interior Waypoint
        //     List.of(new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(23))),
        //     // End 250" forward and 23" left of where we started, facing 135 degrees CCW
        //     new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(23), new Rotation2d(3 * Math.PI / 4)),
        //     config);
        
        // SwerveControllerCommand coralStationCommand = new SwerveControllerCommand(
        //     coralStationTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,
        
        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);
            
        // // Go back to reef trajectory
        // Trajectory reefTrajectory2 = TrajectoryGenerator.generateTrajectory(
        //     // Start at the position of the coral station
        //     new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(23), new Rotation2d(3 * Math.PI / 4)),
        //     // Interior Waypoint
        //     List.of(new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(23))),
        //     // End 137.5" forward and 121.625" left of where we started, facing 150 degrees CCW (at middle of reef)
        //     new Pose2d(Units.inchesToMeters(137.25), Units.inchesToMeters(121.625), new Rotation2d(5 * Math.PI / 6)),
        //     config);
            
        // SwerveControllerCommand reefCommand2 = new SwerveControllerCommand(
        //     reefTrajectory2,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,
            
        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);
        
        // // Shimmy lefgt command
        // Trajectory shimmyLeftTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Interior Waypoint
        //     List.of(new Translation2d(0, -0.1)),
        //     // End 0.17 meters right of where we started, facing forward
        //     new Pose2d(0, 0.17, new Rotation2d(0)),
        //     config);

        // SwerveControllerCommand shimmyLeftCommand = new SwerveControllerCommand(
        //     shimmyLeftTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,

        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);
        

        // // Go to reef -> Lift algae out of reef -> Shimmy to the right to be in line with branch -> Score coral -> Lower elevatorto feeder level
        // // -> Go to coral station -> Wait 3 second -> Return to middle of reeef -> Lift elevator to L3 -> Shimmy left -> Score coral -> Stop
        // return moveToReefCommand.andThen(liftAlgaeCommand.andThen(shimmyRightCommand.andThen(scoreCoralCommand.andThen(lowerElevatorCommand.andThen(
        //     coralStationCommand.withTimeout(3).andThen(reefCommand2.andThen(liftAlgaeCommand).andThen(shimmyLeftCommand
        //     .andThen(scoreCoralCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false))))))))));

    // }

    // // Auto Routine for left side of field
    // public Command leftAutoCommand(TrajectoryConfig config, ProfiledPIDController thetaController) {
        
    //     // Go to correct position on reef to score coral
    //     Trajectory reefTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         List.of(new Pose2d(0, 0, new Rotation2d(0)), 
    //         // Move forward 137.25" and right 21.625"
    //         new Pose2d(Units.inchesToMeters(137.25), -Units.inchesToMeters(21.625), new Rotation2d(0)),
    //         // Move right 100" and turn 150 degrees CW (-5PI/6 radians)
    //         new Pose2d(Units.inchesToMeters(137.25), -Units.inchesToMeters(121.625), new Rotation2d(-5 * Math.PI / 6))),
    //         config);
            
    //     SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
    //         reefTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,
        
    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);
    
    //     // Lift algae out command (Go to level 3)
    //     Command liftAlgaeCommand = m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3);
    
    //     // Shimmy left command
    //     Trajectory shimmyLeftTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Interior Waypoint
    //         List.of(new Translation2d(0, 0.1)),
    //         // End 0.17 meters left of where we started, facing forward
    //         new Pose2d(0, 0.17, new Rotation2d(0)),
    //         config);
    
    //     SwerveControllerCommand shimmyLeftCommand = new SwerveControllerCommand(
    //         shimmyLeftTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,
    
    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);
            
    //     // Score coral command
    //     Command scoreCoralCommand = m_coralSubSystem.reverseIntakeCommand().withTimeout(1);
    
    //     // Reset odometry to the starting pose of the trajectory.
    //     m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    
    //     // Go to reef -> Lift algae out of reef -> Shimmy to left right to be in line with branch -> Score coral -> Stop
    //     return moveToReefCommand.andThen(liftAlgaeCommand.andThen(shimmyLeftCommand.andThen(scoreCoralCommand.andThen(
    //         () -> m_robotDrive.drive(0, 0, 0, false)))));
    //     }
}
