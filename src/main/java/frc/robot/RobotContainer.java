package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Amp;
import frc.robot.commands.FloorIntake;
import frc.robot.commands.HangRetract;
import frc.robot.commands.Shoot;
import frc.robot.commands.Prime;
import frc.robot.commands.TopIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        private final Field2d field;
        private final Drivetrain m_drivetrain = new Drivetrain();
        private final Intake m_intake = new Intake();
        private final Shooter m_shooter = new Shooter();
        private final Hang m_hang = new Hang();

        private final SendableChooser<Command> autoChooser;
        public static class controllers{
                public final static CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
                public final static CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);
        }

        public RobotContainer() {
        field = new Field2d();
        //SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
        
        NamedCommands.registerCommand("Prime", new Prime(m_shooter));
        NamedCommands.registerCommand("Shoot", new Shoot(m_shooter, m_intake));
        NamedCommands.registerCommand("Floor Intake", new FloorIntake(m_intake, 1));
        NamedCommands.registerCommand("Amp", new Amp(m_shooter, m_intake));
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }


    private void configureBindings() {
        // Tank Drive (single controller)
        //FOR DUAL CONTROLLER SETUP COMMENT THIS BLOCK AND UNCOMMENT THE NEXT BLOCK
        /*m_drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> m_drivetrain.drive(controllers.mControls.getLeftY(), controllers.mControls.getRightY()),
                        m_drivetrain));*/

        /* FOR DUAL CONTROLLER SETUP UNCOMMENT THIS BLOCK*/
        //Tank Drive (2 controllers)            
        m_drivetrain.setDefaultCommand(new RunCommand(
                () -> m_drivetrain.drive(controllers.mDriver.getLeftY(), controllers.mDriver.getRightY()),
                m_drivetrain));

        // Amp Shoot
        controllers.mControls

                .b()
                .whileTrue(
                new Amp(m_shooter, m_intake).withTimeout(0.5));

        // Shooter Prime
        controllers.mControls
                .leftTrigger()
                .whileTrue(
                        new Prime(m_shooter));

        // Shooter Launch
        controllers.mControls
                .leftBumper()
                .whileTrue(
                        new Shoot(m_shooter, m_intake));

        // Floor Intake
        controllers.mControls
                .rightTrigger()
                .whileTrue(
                        new FloorIntake(m_intake, 1));

        // Floor Intake Reverse
        controllers.mControls
                .rightBumper()
                .whileTrue(
                        new FloorIntake(m_intake, -1));

        // Top Intake
        controllers.mControls
                .x()
                .whileTrue(
                        new TopIntake(m_shooter));

        // Hang Winch
        controllers.mControls
                .a()
                .whileTrue(
                        new HangRetract(m_hang, HangConstants.speed));

        // Hang Unwinch
        controllers.mControls
                .y()
                .whileTrue(
                        new HangRetract(m_hang, -HangConstants.speed));
        
        /*
        controllers.mControls
        .a()
        .and(controllers.mControls.rightBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        controllers.mControls
                .a()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                */
    }

    public Command getAutonomousCommand() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetGyro();
        return autoChooser.getSelected();
    }
}

