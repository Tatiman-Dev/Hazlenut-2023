// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import static frc.robot.Constants.*;


import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  XboxController exampleXbox = new XboxController(0);
  private WPI_TalonSRX rightMotor;
  private WPI_TalonSRX leftMotor;

  private boolean Detected = false;

  private Pigeon2 m_pigeon = new Pigeon2(0);

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final DigitalInput magnetSensor = new DigitalInput(9);

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private ColorMatchResult match;

  private final Color kBlueTarget = new Color(0.193848, 0.418701, 0.388428);
  private final Color kRedTarget = new Color(0.472412, 0.370605, 0.157227);
  private final Color kCarpetTarget = new Color(0.249023, 0.480713, 0.273926);

  private final double target = 7.2;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   
  @Override
  public void robotInit() {
    rightMotor = new WPI_TalonSRX(rightDrive);
    leftMotor = new WPI_TalonSRX(leftDrive);
    rightMotor.setInverted(true);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kCarpetTarget);

    ShuffleboardTab pigeonTab = Shuffleboard.getTab("Pigeon IMU");
    ShuffleboardTab colorTab = Shuffleboard.getTab("Color Sensor");
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Tab");


    pigeonTab.addNumber("Pitch", () -> m_pigeon.getPitch());
    pigeonTab.addNumber("Yaw", () -> m_pigeon.getYaw());
    pigeonTab.addNumber("Roll", () -> m_pigeon.getRoll());

    colorTab.addString("Detected Color: ", () -> colorCheck());
    colorTab.addBoolean("Detected Bool: ", () -> Detected);

    driveTab.addDouble("Xbox Left X", () -> exampleXbox.getLeftX());
    driveTab.addDouble("Xbox Left Y", () -> exampleXbox.getLeftY());

    driveTab.addBoolean("Magnet", () -> magnetSensor.get());


  }

  public String colorCheck(){
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget && match.confidence > 0.9) {
      colorString = "Blue";
    } else if (match.color == kRedTarget && match.confidence > 0.9) {
      colorString = "Red";
    } else if (match.color == kCarpetTarget && match.confidence > 0.9 ) {
      colorString = "Gray";
    } else {
      colorString = "Unknown/Floor";
    }

    return colorString;
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
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget && match.confidence > 0.9) {
      colorString = "Blue";
    } else if (match.color == kRedTarget && match.confidence > 0.9) {
      colorString = "Red";
    } else if (match.color == kCarpetTarget && match.confidence > 0.9 ) {
      colorString = "Gray";
    } else {
      colorString = "Unknown/Floor";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Confidence", match.confidence);

    



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

  double kP = 0.02;
  double kI = 0;
  double kD = 0;
  double e = 0;
  double ePrev = 0;


  double iAccumulator = 0;
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double pitch = m_pigeon.getPitch();
    double e = target - pitch;

    iAccumulator += 0.02 * e;

    

    double p = kP * e;
    double i = kI * iAccumulator;
    double d = kD * (e - ePrev)/0.02;

    ePrev = e;

    double pid = p + i + d;

    setMotorSpeed(-pid);

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setMotorSpeed(exampleXbox.getLeftY() * -1);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  public void setMotorSpeed(double speed){
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    setMotorSpeed(0.25);
    Detected = false;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);



    if (match.color == kRedTarget && match.confidence > 0.9) {
      Detected = true;
    }
    if (Detected == true){
      setMotorSpeed(0.0);
    }

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
