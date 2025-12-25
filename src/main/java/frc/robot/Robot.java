// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Motores Chassis
  SparkMax left1 = new SparkMax(1, MotorType.kBrushed);
  SparkMax left2 = new SparkMax(2, MotorType.kBrushed);
  SparkMax right1 = new SparkMax(3, MotorType.kBrushed);
  SparkMax right2 = new SparkMax(4, MotorType.kBrushed);
  Encoder leftEncoder = new Encoder(1,2);
  Encoder rightEncoder = new Encoder(3,4);
  AnalogGyro gyro = new AnalogGyro(1);

  // Config Chassis
  SparkMaxConfig l2Config = new SparkMaxConfig();
  SparkMaxConfig r1Config = new SparkMaxConfig();
  SparkMaxConfig r2Config = new SparkMaxConfig();

  // Simulación
  DCMotor motor = DCMotor.getCIM(2);
  double massKg = 60;
  double rMeters = Units.inchesToMeters(6);
  double rbMeters = Units.inchesToMeters(16);
  double JKgMetersSquared = 5.5;
  double gearing = 8.45;
  private DifferentialDrivetrainSim ddsim = new DifferentialDrivetrainSim(
    LinearSystemId.createDrivetrainVelocitySystem(
            motor,
            massKg,
            rMeters,
            rbMeters,
            JKgMetersSquared,
            gearing),
    motor,
    gearing,
    rbMeters,
    rMeters,
    null);
  EncoderSim leftEncSim = new EncoderSim(leftEncoder);
  EncoderSim rightEncSim = new EncoderSim(rightEncoder);
  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
  private Field2d field2d = new Field2d();

  // Differential Drive
  private final DifferentialDrive ddDrive = new DifferentialDrive(left1, right1);
  private Joystick control = new Joystick(0);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    leftEncoder.getDistance(),rightEncoder.getDistance(),
    new Pose2d(5.0,13.5, new Rotation2d()));

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Spark Configs

    l2Config  // Left slave follows main.
          .follow(1);
    r1Config  // Right main inverted.
          .inverted(true);
    r2Config  // Right slave follows main.
          .follow(3);


  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    ddDrive.arcadeDrive(-control.getY(), -control.getX());
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  public void Drivetrain() {
    leftEncoder.setDistancePerPulse(2 * Math.PI * rMeters / 2048);
    rightEncoder.setDistancePerPulse(2 * Math.PI * rMeters / 2048);
    SmartDashboard.putData("Field", field2d);
  }
  public void simulationPeriodic() {
    ddsim.setInputs(
      left1.get() * RobotController.getInputVoltage(),
      -(right1.get() * RobotController.getInputVoltage()));
    ddsim.update(0.02);
    leftEncSim.setDistance(ddsim.getLeftPositionMeters());
    leftEncSim.setRate(ddsim.getLeftVelocityMetersPerSecond());
    rightEncSim.setDistance(ddsim.getRightPositionMeters());
    rightEncSim.setRate(ddsim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-ddsim.getHeading().getDegrees());
  }
  public void periodic() {
    // Get the rotation of the robot from the gyro.
    var gyroAngle = gyro.getRotation2d();
    // Update the pose
    odometry.update(gyroAngle,
                  leftEncoder.getDistance(),
                  rightEncoder.getDistance());
    field2d.setRobotPose(odometry.getPoseMeters());
  }
}
