// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 * @param <Talon>
 */
public class Robot<Talon> extends TimedRobot {
  /**
   * private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
*/
//definitions for the hardware. Change this if ou change what stuff you have plugged in
CANSparkMax driveLeftA= new CANSparkMax(3, MotorType.kBrushed);
CANSparkMax driveLeftB= new CANSparkMax(4, MotorType.kBrushed);
CANSparkMax driveRightA= new CANSparkMax(1, MotorType.kBrushed);
CANSparkMax driveRightB= new CANSparkMax(2, MotorType.kBrushed);
//CANSparkMax arm= new CANSparkMax(5, MotorType.kBrushed);
//VictorSPX intake = new VictorSPX(6);
//PWMTalonSRX drive = new PWMTalonSRX(7);

Joystick driverController = new Joystick(0);

//Constants for controlling the arm. Consider tuning these for your particular robot
final double armHoldUp = 0.08;
final double armHoldDown = 0.13;
final double armTravel = 0.5;

final double armTimeUp = 0.5;
final double armTimeDown = 0.35;

//variables needed for code\
boolean armUp= true; //Arm initialised fo up because that is how it woudld start a match
boolean burstMode = false;
double lastBurstTime= 0;

double autoStart= 0;
boolean goForAuto = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /**
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    */
    CameraServer.startAutomaticCapture();
    
    driveLeftA.setInverted(true);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    driveRightA.burnFlash();
    driveRightB.setInverted(false);
    driveRightB.burnFlash();

    //arm.setInverted(false);
   // arm.setIdleMode(IdleMode.kBrake);
    //arm.burnFlash();

    //add a thing on the dashboard to turn off auto if necessary
    SmartDashboard.putBoolean("Go for Auto", false);
    goForAuto=SmartDashboard.getBoolean("Go for Auto", false);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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

    //get a time for auton start to do events base on time later
    autoStart =Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto=SmartDashboard.getBoolean("Go for Auto", false);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //arm control code. Same as in teleop
    /*if(armUp) {
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldUp);
      }
    }
      **/
      
    

      //get time since start of auto
      double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
      if(goForAuto){
        //series of timed events making up the flow of auto
        if (autoTimeElapsed < 3){
          //spit ball out for three seconds
          //intake.set(ControlMode.PercentOutput,-1);
        }else if (autoTimeElapsed <6){
          //stop spitting out the ball and drive backwards slowly for three seconds
          //intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(-0.3);
        driveLeftB.set(-0.3);
        driveRightA.set(-0.3);
        driveRightB.set(-0.3);
        //drive.set(0.3);

        
        }else{
          //do nothing for the rest of auto
         // intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
        }
              }
      }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(2);

    double driveLeftPower = forward- turn;
    double driveRightPower= forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);

    //intake controls
    if(driverController.getRawButton(5)){
      //intake.set(VictorSPXControlMode.PercentOutput,1);;
    }
    else{
      //intake.set(VictorSPXControlMode.PercentOutput,0);
    }

    //arm controls
    /*if (armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
      }
      else{
        if(Timer.getFPGATimestamp()- lastBurstTime< armTimeDown){
          arm.set(-armTravel);

        }
        else{
          arm.set(-armHoldDown);
        }
      }
      if (driverController.getRawButtonPressed(6) && !armUp){
        lastBurstTime= Timer.getFPGATimestamp();
        armUp = false;
      }
    **/
    

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    //on disable turn off everything
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    //arm.set(0);
    //intake.set(ControlMode.PercentOutput,0);
  }

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
