// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.ApproachReef.LevelOffsets;
import frc.robot.commands.ScoreL1Command;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.canWatchdog.CANWatchdogIOComp;
import frc.robot.subsystems.claw.ClawRollers;
import frc.robot.subsystems.claw.ClawRollers.ClawRollersTarget;
import frc.robot.subsystems.claw.ClawRollersController;
import frc.robot.subsystems.claw.ClawRollersController.ClawState;
import frc.robot.subsystems.claw.ClawRollersIO;
import frc.robot.subsystems.claw.ClawRollersIOSim;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIO;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIO;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIOCANRange;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIOSim;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensors;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorsConstants;
import frc.robot.subsystems.l1_pivot.L1Pivot;
import frc.robot.subsystems.l1_pivot.L1PivotController;
import frc.robot.subsystems.l1_pivot.L1PivotIO;
import frc.robot.subsystems.l1_pivot.L1PivotIOSim;
import frc.robot.subsystems.l1_pivot.L1PivotIOTalonFX;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.rgb.RGBIOCANdle;
import frc.robot.subsystems.superstructure.SuperstructureController;
import frc.robot.subsystems.superstructure.SuperstructureController.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // DO NOT DELETE -- this actually does something important
  private RobotState robotState = RobotState.getInstance();

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  @AutoLogOutput(key = "CommandedOffset")
  private LevelOffsets levelOffsets = LevelOffsets.L1_OFFSET;

  private boolean eject = false;

  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private IntakeRollers intakeRollers;
  private IntakePivot intakePivot;
  private IntakeController intakeController;
  private IntakeSensors intakeSensors;
  private ClawRollers clawRollers;
  private ClawRollersController clawRollersController;

  private SuperstructureController superstructureController;
  private Arm arm;
  private Elevator elevator;
  private L1PivotController l1PivotController;
  private L1Pivot l1Pivot;

  public RobotContainer() {
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          vision = new Vision(new VisionIOPhotonvision(1), new VisionIOPhotonvision(2));
          rgb = new RGB(new RGBIOCANdle());
          canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          l1Pivot = new L1Pivot(new L1PivotIOTalonFX());
          intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
          intakePivot = new IntakePivot(new IntakePivotIOTalonFX());

          intakeSensors =
              new IntakeSensors(
                  new IntakeSensorIOCANRange(IntakeSensorsConstants.PORT_ID_1),
                  new IntakeSensorIOCANRange(IntakeSensorsConstants.PORT_ID_2));

          // elevator = new Elevator(new ElevatorIOTalonFX());
          // arm = new Arm(new ArmIOTalonFX());
        }
        case SIM -> {
          SwerveDriveSimulation driveSimulation = RobotSimState.getInstance().getDriveSimulation();
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          swerve =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0], driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1], driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2], driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3], driveSimulation.getModules()[3]));
          vision =
              new Vision(
                  new VisionIOPhotonvisionSim(1, driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonvisionSim(2, driveSimulation::getSimulatedDriveTrainPose));

          SimulatedArena.getInstance().resetFieldForAuto();

          intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
          intakePivot = new IntakePivot(new IntakePivotIOSim());
          l1Pivot = new L1Pivot(new L1PivotIOSim());
          intakeSensors = new IntakeSensors(new IntakeSensorIOSim(), new IntakeSensorIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          arm = new Arm(new ArmIOSim());
          clawRollers = new ClawRollers(new ClawRollersIOSim());
          SimulatedArena.getInstance().resetFieldForAuto();
        }
      }
    }

    // Swerve
    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // Vision
    if (vision == null) {
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
    }

    // CAN Watchdog
    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    // RGB
    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }
    if (clawRollers == null) {
      clawRollers = new ClawRollers(new ClawRollersIO() {});
    }
    clawRollersController = new ClawRollersController(clawRollers);

    if (intakeRollers == null) {
      intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
    }
    if (intakePivot == null) {
      intakePivot = new IntakePivot(new IntakePivotIO() {});
    }
    if (intakeSensors == null) {
      intakeSensors = new IntakeSensors(new IntakeSensorIO() {}, new IntakeSensorIO() {});
    }
    intakeController = new IntakeController(intakeRollers, intakePivot, intakeSensors);

    if (l1Pivot == null) {
      l1Pivot = new L1Pivot(new L1PivotIO() {});
    }
    l1PivotController = new L1PivotController(l1Pivot);

    // Superstructure
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    superstructureController = new SuperstructureController(elevator, arm);

    clawRollersController = new ClawRollersController(clawRollers);

    nameCommands();
    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    // Register Command Names
    NamedCommands.registerCommand(
        "Score_L1",
        new SequentialCommandGroup(
                new FunctionalCommand(
                    () -> intakeController.setTargetStateCommand(IntakeState.L1),
                    () -> {},
                    (e) -> {},
                    () -> intakeController.intakeReachedTarget(),
                    intakeController))
            .withTimeout(2.6));
    NamedCommands.registerCommand(
        "Eject",
        new InstantCommand(() -> clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_TOP)));
    NamedCommands.registerCommand(
        "Eject_L1",
        new SequentialCommandGroup(intakeController.setTargetStateCommand(IntakeState.EJECT)));
    NamedCommands.registerCommand("Zero", swerve.zeroGyroCommand());
    // NamedCommands.registerCommand(
    //     "L2_Algae",
    //     new InstantCommand(() ->
    // superstructureController.setSuperstructureStateCommand(SuperstructureState.L2_ALGAE)));
  }

  private void configureBindings() {
    configureAlgaeButtons();
    configureOverrideButtons();
    configureMultiUseButtons();
    configureL1Buttons();
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                  if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                    swerve.clearHeadingControl();
                  }
                })
            .withName("Drive Teleop"));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    // driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    driverA.b().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.L1));
    driverA.x().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.INTAKE));
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.IDLE));
    driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));

    driverB
        .leftTrigger()
        .onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.FORCE_INTAKE));
    // driverB
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 superstructureController.setSuperstructureState(
    //                     SuperstructureState.GROUND_ALGAE)));
    // driverB
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //
    // superstructureController.setSuperstructureState(SuperstructureState.L2_ALGAE)));
    // // auto align
    // driverA
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 clawRollersController.setVoltageTarget(
    //                     ClawRollersController.ClawState.EJECT_TOP)));
    // driverA
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 clawRollersController.setVoltageTarget(
    //                     ClawRollersController.ClawState.INTAKE)));

    // driverB
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //
    // superstructureController.setSuperstructureState(SuperstructureState.L1_LEFT)));
    // driverB
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    // superstructureController.setSuperstructureState(SuperstructureState.STOW)));
    // driverB
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //
    // superstructureController.setSuperstructureState(SuperstructureState.L1_RIGHT)));
    // auto align
    driverA
        .leftBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, true, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl())))
                .andThen(intakeController.setTargetStateCommand(IntakeController.IntakeState.L1))
                .andThen(new WaitCommand(1))
                .andThen(
                    intakeController.setTargetStateCommand(
                        IntakeController.IntakeState.UPRIGHT_INTAKE)));
    // auto align
    driverA
        .rightBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, false, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl())))
                .andThen(intakeController.setTargetStateCommand(IntakeController.IntakeState.L1))
                .andThen(new WaitCommand(1))
                .andThen(
                    intakeController.setTargetStateCommand(
                        IntakeController.IntakeState.UPRIGHT_INTAKE)));
  }

  private void configureL1Buttons() {
    driverA.x().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.INTAKE));
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.IDLE));
    driverB
        .leftTrigger()
        .onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.FORCE_INTAKE));
    driverB
        .leftTrigger()
        .onFalse(intakeController.setTargetStateCommand(IntakeController.IntakeState.INTAKE));
    driverB.rightTrigger().onTrue(new ScoreL1Command(intakeController, l1PivotController));
  }

  private void configureMultiUseButtons() {
    // outtake

  }

  private void configureOverrideButtons() {
    driverB
        .a()
        .onTrue(
            superstructureController.setSuperstructureStateCommand(SuperstructureState.ZEROING));
    driverB
        .rightBumper()
        .onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.HOLD));
    driverB
        .leftBumper()
        .onTrue(superstructureController.setSuperstructureStateCommand(SuperstructureState.TOP));
  }

  private void configureAlgaeButtons() {
    driverB
        .povDown()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.GROUND_ALGAE)));
    driverB
        .povRight()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.L2_ALGAE)));
    driverB
        .povLeft()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.L3_ALGAE)));

    driverB
        .povUp()
        .onTrue(
            superstructureController.setSuperstructureStateCommand(
                SuperstructureState.BARGE_RIGHT));

    driverB
        .rightBumper()
        .onTrue(superstructureController.setSuperstructureStateCommand(SuperstructureState.STOW));
    driverA
        .a()
        .onTrue(
            new InstantCommand(
                () ->
                    clawRollersController.setClawTarget(
                        ClawRollersController.ClawState.EJECT_TOP)));
  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround

    BooleanSupplier flipAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance ll flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        flipAlliance,
        swerve);

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance()
        .schedule(new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 5, .5)));

    // vibrate controller at 30 seconds left
    CommandScheduler.getInstance()
        .schedule(
            new WaitCommand(105)
                .andThen(
                    new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 3, 0.4))));
  }

  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    SmartDashboard.putString("Current Auto", autoChooser.get().getName());
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = ((currentAngle - newAngle) % 360 + 360) % 360;
    double b = ((currentAngle - newAngle) % 360 + 360) % 360;
    return a < b ? a : -b;
  }

  public void updateSimulation() {

    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
