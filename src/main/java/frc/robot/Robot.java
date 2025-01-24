// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  int frontLeft = 16, frontRight = 4, rearLeft = 13, rearRight = 1, outtake = 14;
  double flSpeed = 0, frSpeed = 0, rlSpeed = 0, rrSpeed = 0;
  SparkMax flMotor, frMotor, rlMotor, rrMotor, shoot;
  RelativeEncoder flEnc, frEnc, rlEnc, rrEnc;
  MecanumDrive dt;
  MecanumDriveKinematics kinematics;
  AHRS gyro;
  SlewRateLimiter xLimiter, yLimiter, rotLimiter;
  double //flFF = 0.000168,
         flFF = 0.000158,
         frFF = 0.000149,
         rlFF = 0.000158420,
         rrFF = 0.000156;
  double flP = 4e-7,
         frP = 4e-7,
         rlP = 4e-7,
         rrP = 4e-7;
  double flI = 1e-7,
         frI = 1e-7,
         rlI = 1e-7,
         rrI = 1e-7;
  double flD = 3,
         frD = 3,
         rlD = 3,
         rrD = 3;

  ProfiledPIDController flPID, frPID, rlPID, rrPID;
  ElevatorFeedforward flFFC;

  Timer timer;


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

    flMotor = new SparkMax(frontLeft, MotorType.kBrushless);
    frMotor = new SparkMax(frontRight, MotorType.kBrushless);
    rlMotor = new SparkMax(rearLeft, MotorType.kBrushless);
    rrMotor = new SparkMax(rearRight, MotorType.kBrushless);
    shoot = new SparkMax(outtake, MotorType.kBrushed);

    SparkMaxConfig config = new SparkMaxConfig();
    /*config.apply(new EncoderConfig().positionConversionFactor(Units.inchesToMeters(6) * Math.PI / 5.95));
    config.apply(new EncoderConfig().velocityConversionFactor(0.1524 / Math.PI * 60));  */
    config.smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .closedLoopRampRate(0.1)
          .closedLoop.pidf(flP, flI, flD, flFF)
                     .maxMotion.maxAcceleration(4000)
                               .maxVelocity(4000)
                               .allowedClosedLoopError(0.005)
                               .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); 

    flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.apply(new SparkMaxConfig().closedLoop.pidf(rlP, rlI, rlD, rlFF));
    rlMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.apply(new SparkMaxConfig().inverted(true));

    config.apply(new SparkMaxConfig().closedLoop.pidf(frP, frI, frD, frFF));
    frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.apply(new SparkMaxConfig().closedLoop.pidf(rrP, rrI, rrD, rrFF));
    rrMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.apply(new SparkMaxConfig().inverted(false));
    shoot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flEnc = flMotor.getEncoder();
    frEnc = frMotor.getEncoder();
    rlEnc = rlMotor.getEncoder();
    rrEnc = rrMotor.getEncoder();
    
    /*flFeed = new SimpleMotorFeedforward(0.002828436, 0.000165385);
    frFeed = new SimpleMotorFeedforward(0.00237629, 0.000166560);
    rlFeed = new SimpleMotorFeedforward(flFF, kDefaultPeriod);
    rrFeed = new SimpleMotorFeedforward(flFF, kDefaultPeriod);*/

    double wheelBase = Units.inchesToMeters(20.75);
    double trackWidth = Units.inchesToMeters(19.5);
    Translation2d flLocation = new Translation2d(wheelBase / 2, trackWidth / 2);
    Translation2d frLocation = new Translation2d(wheelBase / 2, -trackWidth / 2);
    Translation2d rlLocation = new Translation2d(-wheelBase / 2, trackWidth / 2);
    Translation2d rrLocation = new Translation2d(-wheelBase / 2, -trackWidth / 2);

    //dt = new MecanumDrive(flMotor::set, rlMotor::set, frMotor::set, rrMotor::set);
    kinematics = new MecanumDriveKinematics(flLocation, frLocation, rlLocation, rrLocation);

    //shoot = new SparkMax(outtake, SparkLowLevel.MotorType.kBrushless);
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();

    timer = new Timer();

    /*
    flPID = new ProfiledPIDController(flP, 0, 0, new TrapezoidProfile.Constraints(50, 100));
    flFFC = new ElevatorFeedforward(0, 0, 1/5676, 0);
    frPID = new ProfiledPIDController(frP, 0, 0, new TrapezoidProfile.Constraints(5, 1));
    rlPID = new ProfiledPIDController(rlP, 0, 0, new TrapezoidProfile.Constraints(5, 1));
    rrPID = new ProfiledPIDController(rrP, 0, 0, new TrapezoidProfile.Constraints(5, 1));
    /
    flPID = new PIDController(flP, 0, 0);
    frPID = new PIDController(frP, 0, 0);
    rlPID = new PIDController(rlP, 0, 0);
    rrPID = new PIDController(rrP, 0, 0);
    /**/
    
    flEnc.setPosition(0);
    frEnc.setPosition(0);
    rlEnc.setPosition(0);
    rrEnc.setPosition(0);

    xLimiter = new SlewRateLimiter(10);
    yLimiter = new SlewRateLimiter(10);
    rotLimiter = new SlewRateLimiter(10);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative, double periodSeconds){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds.discretize(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d())
                      : chassisSpeeds,
      periodSeconds
      )
    );
    wheelSpeeds.desaturate(7);

    flSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontLeftMetersPerSecond, Units.inchesToMeters(6));
    frSpeed = Util.metersPerSecondToRPM(wheelSpeeds.frontRightMetersPerSecond, Units.inchesToMeters(6));
    rlSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearLeftMetersPerSecond, Units.inchesToMeters(6));
    rrSpeed = Util.metersPerSecondToRPM(wheelSpeeds.rearRightMetersPerSecond, Units.inchesToMeters(6));

    if (Math.abs(xSpeed) < 2.8 && Math.abs(ySpeed) > 3.5){
      frSpeed = - flSpeed;
      rlSpeed = - rrSpeed;
    }

    if (Math.abs(ySpeed) < 2.1 && Math.abs(xSpeed) > 3.5){
      frSpeed = flSpeed;
      rlSpeed = rrSpeed;
    }
  }

  /*public void driveYeah(){
    //if (Util.checkDriverDeadband(driver.getLeftY()) || Util.checkDriverDeadband(driver.getLeftX()) || Util.checkDriverDeadband(driver.getRightX())){
      drive(- (Util.checkDriverDeadband(driver.getLeftY()) ? Math.pow(driver.getLeftY(), 3) * 28.6124 : 0), 
            - (Util.checkDriverDeadband(driver.getLeftX()) ? Math.pow(driver.getLeftX(), 3) * 14.6124 : 0), 
            - (driver.getRightX() * Math.abs(driver.getRightX())),
               false);
    //}
  }*/

  public void test(){
    //flSpeed = 0.5;
    //frSpeed = 1000;
    //rlSpeed = 0.5;
    //rrSpeed = 1000;

    /*flPID.setGoal(5 * 5.95);
    frPID.setGoal(5 * 5.95);
    rlPID.setGoal(5 * 5.95);
    rrPID.setGoal(5 * 5.95);*/
    /*flPID.setSetpoint(0.5);
    frPID.setSetpoint(0.5);
    rlPID.setSetpoint(0.5);
    rrPID.setSetpoint(0.5);*/


  }

  public void stop(){
    flSpeed = 0;
    frSpeed = 0;
    rlSpeed = 0;
    rrSpeed = 0;
    /*flMotor.stopMotor();
    frMotor.stopMotor();
    rlMotor.stopMotor();
    rrMotor.stopMotor();*/
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
    if (Util.checkDriverDeadband(driver.getLeftY()) || Util.checkDriverDeadband(driver.getLeftX()) || Util.checkDriverDeadband(driver.getRightX())){
      drive(
        - yLimiter.calculate(driver.getLeftY()) * 7,
        - xLimiter.calculate(driver.getLeftX()) * 7,
        - rotLimiter.calculate(driver.getRightX()) * 5,
        false,
        getPeriod()
      );
    } else { stop(); }
    //test();
    //dt.driveCartesian(driver.getLeftY(), - driver.getLeftX(), driver.getRightX());*/
    
    //flMotor.setVoltage(1.57);

    flMotor.getClosedLoopController().setReference(flSpeed, ControlType.kMAXMotionVelocityControl);
    frMotor.getClosedLoopController().setReference(frSpeed, ControlType.kMAXMotionVelocityControl);
    rlMotor.getClosedLoopController().setReference(rlSpeed, ControlType.kMAXMotionVelocityControl);
    rrMotor.getClosedLoopController().setReference(rrSpeed, ControlType.kMAXMotionVelocityControl);

    /*flPID.setGoal(1000);
    //flMotor.setVoltage(flPID.calculate(flEnc.getVelocity()) + flFFC.calculate(flPID.getSetpoint().position));
    flPID.calculate(flEnc.getVelocity());
    flMotor.setVoltage(flFFC.calculate(flPID.getSetpoint().position));
    frMotor.setVoltage(frPID.calculate(frEnc.getVelocity()));
    rlMotor.setVoltage(rlPID.calculate(rlEnc.getVelocity()));
    rrMotor.setVoltage(rrPID.calculate(rrEnc.getVelocity()));

    /*flMotor.setVoltage(flFeed.calculate(flSpeed));
    frMotor.setVoltage(frFeed.calculate(frSpeed));*/

    if (driver.getR2Button()){
      shoot.set(0.3);
    } else if (driver.getL2Button()){
      shoot.set(-0.3);
    } else {
      shoot.set(0);
    }

    SmartDashboard.putNumber("flVel", flMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("frVel", frMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("rlVel", rlMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("rrVel", rrMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("flCurrent", flMotor.getOutputCurrent());
    SmartDashboard.putNumber("frCurrent", frMotor.getOutputCurrent());
    SmartDashboard.putNumber("rlCurrent", rlMotor.getOutputCurrent());
    SmartDashboard.putNumber("rrCurrent", rrMotor.getOutputCurrent());
    /*SmartDashboard.putNumber("flVel", Util.metersPerSecondToRPM(flMotor.getEncoder().getVelocity(), Units.inchesToMeters(6)));
    SmartDashboard.putNumber("frVel", Util.metersPerSecondToRPM(frMotor.getEncoder().getVelocity(), Units.inchesToMeters(6)));
    SmartDashboard.putNumber("rlVel", Util.metersPerSecondToRPM(rlMotor.getEncoder().getVelocity(), Units.inchesToMeters(6)));
    SmartDashboard.putNumber("rrVel", Util.metersPerSecondToRPM(rrMotor.getEncoder().getVelocity(), Units.inchesToMeters(6)));*/
    SmartDashboard.putNumber("flPosition", flEnc.getPosition());
    SmartDashboard.putNumber("frPosition", frEnc.getPosition());
    SmartDashboard.putNumber("rlPosition", rlEnc.getPosition());
    SmartDashboard.putNumber("rrPosition", rrEnc.getPosition());
    SmartDashboard.putNumber("flSetPoint", flSpeed * 5.95);
    SmartDashboard.putNumber("frSetPoint", frSpeed * 5.95);
    SmartDashboard.putNumber("rlSetPoint", rlSpeed * 5.95);
    SmartDashboard.putNumber("rrSetPoint", rrSpeed * 5.95);
    SmartDashboard.putNumber("xSpeed", driver.getLeftY());
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("gyro", gyro.getAngle() % 360);
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
    //m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }*/
    

    if (timer.get() <= 3 && timer.get() >= 1){
      drive(0, -0.5, 0, false, getPeriod());
    }
    if (timer.get() > 3 && timer.get() < 5){
      stop();
    }
    if (timer.get() <= 7 && timer.get() >= 5){
      drive(0, 0.5, 0, false, getPeriod());
    }
    if (timer.get() > 7){
      stop();
    }

    if (timer.get() <= 10 && timer.get() >= 8){
      drive(0, -0.5, 0, false, getPeriod());
    }
    if (timer.get() > 10 && timer.get() < 12){
      stop();
    }
    if (timer.get() >= 12 && timer.get() <= 14){
      drive(0, 0.5, 0, false, getPeriod());
    }
    if (timer.get() > 14){
      stop();
    }

    if (timer.get() <= 17 && timer.get() >= 15){
      drive(0, -0.5, 0, false, getPeriod());
    }
    if (timer.get() > 17 && timer.get() < 19){
      stop();
    }
    if (timer.get() >= 19 && timer.get() <= 21){
      drive(0, 0.5, 0, false, getPeriod());
    }
    if (timer.get() > 21){
      stop();
    }
  
    /*flMotor.getClosedLoopController().setReference(flSpeed, ControlType.kMAXMotionVelocityControl);
    frMotor.getClosedLoopController().setReference(frSpeed, ControlType.kMAXMotionVelocityControl);
    rlMotor.getClosedLoopController().setReference(rlSpeed, ControlType.kMAXMotionVelocityControl);
    rrMotor.getClosedLoopController().setReference(rrSpeed, ControlType.kMAXMotionVelocityControl);*/
    //}
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //dt.driveCartesian(driver.getLeftY(), driver.getLeftX(), driver.getRightX());
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
