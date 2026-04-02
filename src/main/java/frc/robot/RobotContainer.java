// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.Chassis.AutoHubToggle;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Climber.Basic.BasicClimberDown;
import frc.robot.commands.Climber.Basic.BasicClimberUp;
import frc.robot.commands.Feeder.Basic.ReverseFeederBasic;
import frc.robot.commands.Feeder.Basic.RunFeederBasic;
import frc.robot.commands.Feeder.PID.RunFeeder;
import frc.robot.commands.Hopper.Basic.ReverseHopperHorizontal;
import frc.robot.commands.Hopper.Basic.RunHopperHorizontal;
import frc.robot.commands.Hopper.PID.RunHopper;
import frc.robot.commands.Intake.Basic.BasicPivotIn;
import frc.robot.commands.Intake.Basic.BasicPivotOut;
import frc.robot.commands.Intake.Basic.ReverseIntakeBasic;
import frc.robot.commands.Intake.Basic.RunIntakeBasic;
import frc.robot.commands.Intake.PID.PivotHalf;
import frc.robot.commands.Intake.PID.PivotIn;
import frc.robot.commands.Intake.PID.PivotOut;
import frc.robot.commands.Intake.PID.RunIntake;
import frc.robot.commands.Shooter.Basic.ReverseShooter;
import frc.robot.commands.Shooter.Basic.RunShooter;
import frc.robot.commands.Shooter.PID.AutoRev;
import frc.robot.commands.Shooter.PID.Rev;
import frc.robot.commands.Shooter.PID.RevToVelocity;
import frc.robot.commands.Shooter.PID.RevWithPower;
import frc.robot.commands.ShooterHood.Basic.ShooterHoodDown;
import frc.robot.commands.ShooterHood.Basic.ShooterHoodUp;
import frc.robot.commands.ShooterHood.PID.AutoAim;
import frc.robot.commands.ShooterHood.PID.HoodToSetpoint;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Timer timer = new Timer();
  private final double deadband = 0.05 * Constants.Swerve.maxSpeed;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(Constants.Swerve.maxSpeed * 0.05).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.09) // Add a 10% deadband
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity); // Use velocity control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain driveTrain = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  private final SendableChooser<Command> autoChooser;

  private final Climber climber;
  private final Feeder feeder;
  private final Hopper hopper;
  private final Intake intake;
  private final Shooter shooter;
  private final ShooterHood shooterHood;
  private final Limelight limelight;
  private final LEDs leds;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    climber = new Climber();
    feeder = new Feeder();
    hopper = new Hopper();
    intake = new Intake();
    shooterHood = new ShooterHood();
    shooter = new Shooter();
    limelight = new Limelight();
    leds = new LEDs(shooter, intake);


    NamedCommands.registerCommand("Run Feeder Basic", new RunFeederBasic(feeder));

    NamedCommands.registerCommand("Run Hopper", new RunHopperHorizontal(hopper, shooter, shooterHood, driveTrain));

    NamedCommands.registerCommand("Run Shooter", new RunShooter(shooter));
    NamedCommands.registerCommand("Rev Velocity", new RevToVelocity(shooter, driveTrain, shooterHood));
    NamedCommands.registerCommand("Shooting Sequence",
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new RunFeeder(feeder, shooter, shooterHood, driveTrain),
        new RunHopper(hopper, shooter, shooterHood, driveTrain),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new PivotHalf(intake)
        )
      ),
      new AutoRev(shooter, driveTrain, shooterHood)));

    NamedCommands.registerCommand("Run Intake", new RunIntake(intake));

    NamedCommands.registerCommand("Pivot Out", new PivotOut(intake));
    NamedCommands.registerCommand("Pivot In", new PivotIn(intake));

    NamedCommands.registerCommand("Shooter Hood Down", new ShooterHoodDown(shooterHood));
    NamedCommands.registerCommand("Hood To Setpoint", new HoodToSetpoint(shooterHood));

    NamedCommands.registerCommand("Hub Toggle", new AutoHubToggle(driveTrain, driverController, drive, shooter, shooterHood));
    
    // Configure the trigger bindings
    configureBindings();
    exportSmartDashboardData();

    //Do named commands before this line
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link CommandPS4Controller
   * PS4} controllers or {@link CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    
    //limelight.setDefaultCommand(new UpdateOdoFromVision(driveTrain, limelight, logger));
    driveTrain.setDefaultCommand(new TeleopDrive(driveTrain, driverController, drive, shooter, shooterHood));
    shooterHood.setDefaultCommand(
      new SequentialCommandGroup(
        new ShooterHoodDown(shooterHood),
        new AutoAim(shooterHood, driveTrain)));

        // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    //operatorController.back().and(operatorController.y()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //operatorController.back().and(operatorController.x()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //operatorController.start().and(operatorController.y()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //operatorController.start().and(operatorController.x()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    //operatorController.back().and(operatorController.y()).whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //operatorController.back().and(operatorController.x()).whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //operatorController.start().and(operatorController.y()).whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //operatorController.start().and(operatorController.x()).whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    driverController.R2().whileTrue(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new RunFeeder(feeder, shooter, shooterHood, driveTrain),
        new RunHopper(hopper, shooter, shooterHood, driveTrain),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new PivotHalf(intake)
        )
      ),
      new AutoRev(shooter, driveTrain, shooterHood)));

    //driverController.povLeft().whileTrue(new Rev(shooter));

    //driverController.R1().whileTrue(new ShooterHoodDown(shooterHood));
    //driverController.L2().whileTrue(new ReverseShooter(shooter));

    //driverController.options().whileTrue(new HoodToSetpoint(shooterHood));
    //driverController.povLeft().whileTrue(new ShooterHoodUp(shooterHood));
    //driverController.povDown().whileTrue(new ShooterHoodDown(shooterHood));

    //driverController.L1().whileTrue(new RunFeederBasic(feeder));
    //driverController.L2().whileTrue(new ReverseFeederBasic(feeder));

    //driverController.square().whileTrue(new RunHopperHorizontal(hopper));
    //driverController.circle().whileTrue(new ReverseHopperHorizontal(hopper));

    //Binded Buttons Currently: R2:Shooting, L1:Outtaking, L2:Intaking, Commented Out: Cross:Shake_Hopper, Circle:Shake_Intake

    // driverController.circle().whileTrue(new SequentialCommandGroup(
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(0.1)
    //     , new intakeReverseShake(intake)),
    //     new ParallelDeadlineGroup(
    //     new WaitCommand(0.1)
    //     , new RunIntake(intake))
    // ));

    // driverController.cross().whileTrue(new SequentialCommandGroup(
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(0.1)
    //     , new ReverseHopperHorizontal(hopper)),
    //     new ParallelDeadlineGroup(
    //     new WaitCommand(0.1)
    //     , new RunHopperHorizontal(hopper))
    // ));

    driverController.povLeft().whileTrue(new PivotIn(intake));
    driverController.povDown().whileTrue(new BasicPivotIn(intake));
    driverController.povRight().whileTrue(new PivotOut(intake));
    driverController.L2().whileTrue(new RunIntake(intake));
    driverController.circle().whileTrue(new RunHopper(hopper, shooter, shooterHood, driveTrain));
    driverController.L1().whileTrue(
      new ParallelCommandGroup(
        new ReverseHopperHorizontal(hopper),
        new ReverseIntakeBasic(intake)
      ));

    driverController.options().whileTrue(new BasicClimberUp(climber));
    driverController.create().whileTrue(new BasicClimberDown(climber));

    operatorController.rightTrigger().whileTrue(new Rev(shooter));
    operatorController.rightBumper().whileTrue(
      new ParallelCommandGroup(
        new RunHopper(hopper, shooter, shooterHood, driveTrain),
        new RunFeeder(feeder, shooter, shooterHood, driveTrain)
      ));
    operatorController.leftTrigger().whileTrue(new RunIntake(intake));
    operatorController.leftBumper().whileTrue(new ReverseIntakeBasic(intake));
    operatorController.povLeft().whileTrue(new BasicPivotIn(intake));
    operatorController.povRight().whileTrue(new BasicPivotOut(intake));
    operatorController.x().whileTrue(new BasicClimberDown(climber));
    operatorController.b().whileTrue(new BasicClimberUp(climber));
    operatorController.povUp().whileTrue(new ShooterHoodUp(shooterHood));
    operatorController.povDown().whileTrue(new ShooterHoodDown(shooterHood));

    // reset the field-centric heading on left bumper press
    driverController.povUp().onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldCentric()));

    driveTrain.registerTelemetry(logger::telemeterize);
  }

  public void exportSmartDashboardData() {
    SmartDashboard.putNumber("Auton Delay", 0);

    SmartDashboard.putData(logger.getField());

    SmartDashboard.putData(climber);
    SmartDashboard.putData(feeder);
    SmartDashboard.putData(hopper);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(shooterHood);
  }

  public void intakeReset() {intake.setZeroed(false);}
  public void intakeResetPos() {intake.intakeResetPos();}
  public void hoodReset() {shooterHood.setZeroed(false);}
  public void hoodDown() {CommandScheduler.getInstance().schedule(new ShooterHoodDown(shooterHood));}

  public void updateOdoFromVision() {
    limelight.updateOdo(driveTrain);
  }
  public void updateDisabledOdoFromVision() {
    limelight.updateDisabledOdo(driveTrain);
  }

  public void setDisabledDeviations() {
    limelight.setRobotHeadingReset(false);
    limelight.disabledDeviations(driveTrain);
  }
  public void setEnabledDeviations() {
    limelight.setRobotHeadingReset(true);
    limelight.enabledDeviations(driveTrain);
  }

  public Command pick() {
    return autoChooser.getSelected();
  }
  public double getAutonDelay() {return SmartDashboard.getNumber("Auton Delay", 0);}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
