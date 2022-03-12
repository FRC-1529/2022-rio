// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <ctre/Phoenix.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "rev/CANSparkMax.h"
/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  // 3 4 0 1
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_frontLeftMotor{2};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_backLeftMotor{4};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_frontRightMotor{0};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_backRightMotor{1};
  frc::PWMSparkMax m_intake{0};
  frc::PWMSparkMax m_pickup{2};
  frc::PWMSparkMax m_flyWheelBottom{1};
  frc::PWMSparkMax m_flywheelTop{3};
  frc::PWMSparkMax m_climbLeft{4};
  frc::PWMSparkMax m_climbRight{5};	
  frc::PWMSparkMax m_turret{6};
  frc::MotorControllerGroup m_leftMotors{m_frontLeftMotor, m_backLeftMotor};
  frc::MotorControllerGroup m_rightMotors{m_frontRightMotor, m_backRightMotor};
  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController m_driveController{0};
  frc::XboxController m_operatorController{1};
  frc::Joystick m_driveControllerBottom{0};
  frc::Joystick m_driveControllerTop{1};
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM}; 
  frc::DoubleSolenoid IntakeSolenoidPCM{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  static const int intakeMotorID = 1, slopedIntakeMotorID = 2, lowerFlywheelID = 3, upperFlywheelID = 4, ClimbLeftID = 5, ClimbRightID = 6;
  rev::CANSparkMax intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax slopedIntakeMotor{slopedIntakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax lowerFlywheel{lowerFlywheelID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax upperFlywheel{upperFlywheelID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ClimbLeft{ClimbLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ClimbRight{ClimbRightID, rev::CANSparkMax::MotorType::kBrushless};
 
 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.SetInverted(true);
  }

  void TeleopPeriodic() override {
    // Drive with tank or arcade style
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftY()*0.75, m_driveController.GetRightX()*-0.75);
    //m_robotDrive.TankDrive(m_driveController.GetLeftY()*0.75, m_driveController.GetRightY()*0.75);

    // double left_power = 0.8*m_driveControllerLeft.GetY();
    //double right_power = 0.8*m_driveControllerRight.GetY();
    //pcmCompressor.EnableDigital();
    //Operates the pneumatics that lifts or sets down our ground intake
    if(m_operatorController.GetLeftBumper() or m_driveController.GetLeftBumper()) {
      IntakeSolenoidPCM.Set(frc::DoubleSolenoid::Value::kReverse);
    } else if (m_operatorController.GetRightBumper() or m_driveController.GetRightBumper()) {
      IntakeSolenoidPCM.Set(frc::DoubleSolenoid::Value::kForward);
    }

   // const double motor_speed = 0.9;
   //Operates the flywheels that allows us to shoot
    lowerFlywheel.Set(m_operatorController.GetRightTriggerAxis()*-1);
    upperFlywheel.Set(m_operatorController.GetRightTriggerAxis());
  //Operates the ground intake to manipulate cargo
    if(m_operatorController.GetAButton()){
      intakeMotor.Set(0.7);
     }else if(m_operatorController.GetXButton()){
      intakeMotor.Set(-1);
     }
     else {
       intakeMotor.Set(0.0);
     }     
  //Operates the sloped intake to manipulate the cargo within the robot   
    if(m_operatorController.GetBButton())
      {
      slopedIntakeMotor.Set(0.5);
      }else if(m_operatorController.GetXButton()){
        slopedIntakeMotor.Set(-0.5);
      }
      else{
      slopedIntakeMotor.Set(0.0);
      }
  //Operates the climb subsystem 
  ClimbLeft.Set(m_operatorController.GetLeftY()*0.25);
  ClimbRight.Set(m_operatorController.GetRightY()*-0.25);
  
  //Operates the turret
  if (m_operatorController.GetRightTriggerAxis())
  {
    m_turret.Set(0.05);
  }
  else if (m_operatorController.GetLeftTriggerAxis())
  {
    m_turret.Set(-0.05);
  }
  else
  {
    m_turret.Set(0);
  }
     

   /// lowerFlywheel.Set(m_operatorController.GetLeftY()*-1);
   //if (m_flywheelTop.Set(m_operatorController.GetLeftY()>.1){
  ///  upperFlywheel.Set(m_operatorController.GetLeftY());
    //m_robotDrive.ArcadeDrive(left_power, right_power);
 // }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
