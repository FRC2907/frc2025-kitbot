// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



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

  int frontLeft = 1, frontRight = 16, rearLeft = 4, rearRight = 13, outtake = 5;
  double flSpeed = 0, frSpeed = 0, rlSpeed = 0, rrSpeed = 0;
  SparkMax flMotor, frMotor, rlMotor, rrMotor;
  MecanumDrive dt;
  MecanumDriveKinematics kinematics;
  AHRS gyro;

  SparkMax shoot;

  PS5Controller driver = new PS5Controller(0);
  PS5Controller operator = new PS5Controller(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    flMotor = new SparkMax(frontLeft, SparkLowLevel.MotorType.kBrushless);
    frMotor = new SparkMax(frontRight, SparkLowLevel.MotorType.kBrushless);
    rlMotor = new SparkMax(rearLeft, SparkLowLevel.MotorType.kBrushless);
    rrMotor = new SparkMax(rearRight, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .closedLoop.pidf(0.00, 0, 0.00, 0.005)
                     .maxMotion.maxAcceleration(20)
                               .maxVelocity(4000);
    flMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rlMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = leftConfig;
    rightConfig.inverted(true);
    frMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rrMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    double wheelBase = 20.25;
    double trackWidth = 19.5;
    Translation2d flLocation = new Translation2d(wheelBase / 2, trackWidth / 2);
    Translation2d frLocation = new Translation2d(wheelBase / 2, -trackWidth / 2);
    Translation2d rlLocation = new Translation2d(-wheelBase / 2, trackWidth / 2);
    Translation2d rrLocation = new Translation2d(-wheelBase / 2, -trackWidth / 2);

    //dt = new MecanumDrive(flMotor::set, rlMotor::set, frMotor::set, rrMotor::set);
    kinematics = new MecanumDriveKinematics(flLocation, frLocation, rlLocation, rrLocation);

    //shoot = new SparkMax(outtake, SparkLowLevel.MotorType.kBrushless);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation / 10);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    flSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, Units.inchesToMeters(6));
    frSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, Units.inchesToMeters(6));
    rlSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, Units.inchesToMeters(6));
    rrSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, Units.inchesToMeters(6));
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
    /*drive(- driver.getLeftY(), - driver.getLeftX(), - driver.getRightX());
    
    flMotor.getClosedLoopController().setReference(flSpeed, ControlType.kMAXMotionVelocityControl);
    frMotor.getClosedLoopController().setReference(frSpeed, ControlType.kMAXMotionVelocityControl);
    rlMotor.getClosedLoopController().setReference(rlSpeed, ControlType.kMAXMotionVelocityControl);
    rrMotor.getClosedLoopController().setReference(rrSpeed, ControlType.kMAXMotionVelocityControl);*/

    flMotor.set(-0.2);

    SmartDashboard.putNumber("flVel", flMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("frVel", frMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("rlVel", rlMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("rrVel", rrMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("flSetPoint", flSpeed);
    SmartDashboard.putNumber("frSetPoint", frSpeed);
    SmartDashboard.putNumber("rlSetPoint", rlSpeed);
    SmartDashboard.putNumber("rrSetPoint", rrSpeed);
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
    //dt.driveCartesian(driver.getLeftY(), driver.getLeftX(), driver.getRightX());

    if (operator.getR2Button()){
      shoot.set(0.5);
    }
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
  @Override
  public void simulationPeriodic() {}
}
