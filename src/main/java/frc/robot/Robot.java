// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//-52 elevtor


package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import java.text.BreakIterator;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers.ControllerMap;
import frc.robot.Controllers.DriverController;
import frc.robot.Controllers.OperatorController;

public class Robot extends TimedRobot {
  private final CANSparkMax m_leftMotor_1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor_2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);

  private final CANSparkMax m_rightMotor_1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor_2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, MotorType.kBrushless);

  private final double 
    m_drive_speed = 1, 
    m_turn_speed = 0.75;

  


  private final CANSparkMax e_motor_1 = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax e_motor_2 = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_ID_2, MotorType.kBrushless);

  private final CANSparkMax intake_pivot = new CANSparkMax(RobotMap.INTAKE_PIVOT_ID, MotorType.kBrushless);
  private final CANSparkMax intake = new CANSparkMax(RobotMap.INTAKE_ID, MotorType.kBrushless);

  //private final DigitalInput  e_top_limit = new DigitalInput(RobotMap.LIMIT_SWITCH_ID_1);
  //private final DigitalInput  e_bottom_limit = new DigitalInput(RobotMap.LIMIT_SWITCH_ID_2);

  private final MotorControllerGroup m_leftMotor = new MotorControllerGroup(m_leftMotor_1, m_leftMotor_2);
  private final MotorControllerGroup m_rightMotor = new MotorControllerGroup(m_rightMotor_1, m_rightMotor_2);

  private final MotorControllerGroup e_motors = new MotorControllerGroup(e_motor_1, e_motor_2);

  public final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  //private final Encoder elevator_encoder = new Encoder(0, 1);

  private static final PigeonIMU _pigeon = new PigeonIMU(2);

  //double[] ypr = new double[3];

  // private final RelativeEncoder shoulder_encoder = shoulder.getEncoder();
  private final RelativeEncoder 
    pivot_encoder = intake_pivot.getEncoder(),
    elevator_encoder = e_motor_1.getEncoder();
    
    //drive_encoder_r = m_rightMotor_1.getEncoder(),
    //drive_encoder_l = m_leftMotor_1.getEncoder();

  private final DriverController d_control = new DriverController(ControllerMap.DRIVER_PORT);
  private final OperatorController o_control = new OperatorController(ControllerMap.OPERATOR_PORT);

  private static final String kDefaultAuto = "Default";
  private static final String kMiddleAuto = "Middle";

  private int current_timer = 0;
  private int cnt = 0;

  private double 
    
    pivot_pos_default = 0,
    pivot_limit_far = 270,
    pivot_limit_close = 0,
    pivot_pos = pivot_pos_default,
    slow_distance = 20,
    
    elevator_pos_max = -53,

    cube_del_0 = 0,
    cube_del_1 = 1,
    cube_del_2 = 2,
    cube_get_elevator = -32,
    cube_get_pivot = 23,

    //rotation_var = 0,
    
    def_co = 0.3,
    slow_co = 0.15,
    coefficient = def_co;

  private final SendableChooser<String> chooser = new SendableChooser<>();

  private final AutoBalancer balancer = new AutoBalancer(m_robotDrive, _pigeon.getPitch());

  private int seq = 0, tel_seq = 0, led_length = 83, led_ms = 0, led_seq = 1;
  private double additive = 1;

  private boolean finished_climb = false;

  private final SpeedLerp 
    pivot_lerp = new SpeedLerp(intake_pivot, pivot_encoder, 0, 1),
    elevator_lerp = new SpeedLerp(e_motors, elevator_encoder, 0, 1),
    
    //MODS
    drive_lerp_r = new SpeedLerp(m_rightMotor, elevator_encoder, 0, 1),
    drive_lerp_l = new SpeedLerp(m_leftMotor, elevator_encoder, 0, 1);
    
    //drive_lerp = new SpeedLerp(, elevator_encoder, 0, 1); // experimental


  private AddressableLED LED; 
  private AddressableLEDBuffer LED_buffer; 

  @Override
  public void robotInit() {
    m_leftMotor.setInverted(true);

    intake_pivot.setInverted(true);

    // MODS
    //drive_encoder_l.setInverted(true);
    //drive_encoder_r.setInverted(true);

    m_leftMotor_1.setIdleMode(IdleMode.kBrake);
    m_leftMotor_2.setIdleMode(IdleMode.kBrake);
    m_rightMotor_1.setIdleMode(IdleMode.kBrake);
    m_rightMotor_2.setIdleMode(IdleMode.kBrake);

    intake_pivot.setIdleMode(IdleMode.kBrake);

    e_motor_1.setIdleMode(IdleMode.kBrake);
    e_motor_2.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);

    ////drive_encode_r.setReverseDirection(true);

    chooser.setDefaultOption("Nothing", kDefaultAuto);
    chooser.addOption("Middle", kMiddleAuto);
    SmartDashboard.putData("Auto choices", chooser);

    //drive_encode_l.setDistancePerPulse(RobotMap.CYCLES_PER_INCH);

    pivot_encoder.setPosition(0);
    elevator_encoder.setPosition(0);

    //drive_encode_r.setDistancePerPulse(RobotMap.CYCLES_PER_INCH);

    SmartDashboard.updateValues();

    LED = new AddressableLED(0);
    LED_buffer= new AddressableLEDBuffer(led_length);

    LED.setLength(led_length);

    LED.start();

    for(int i = 0; i < LED_buffer.getLength(); i++){
      LED_buffer.setHSV(i, 0, 255, 255);
    }

    LED.setData(LED_buffer);
  }

  @Override
  public void robotPeriodic() {




      super.robotPeriodic();

      //_pigeon.getYawPitchRoll(ypr);


      SmartDashboard.putNumber("drive_left_speed", m_leftMotor.get());
      SmartDashboard.putNumber("drive_right_speed", m_rightMotor.get());

      SmartDashboard.putNumber("Intake_speed", intake.get());
      SmartDashboard.putNumber("pivot_speed", intake_pivot.get());

      SmartDashboard.putNumber("pivot position", pivot_encoder.getPosition());
      SmartDashboard.putNumber("elevator position", elevator_encoder.getPosition());

      //SmartDashboard.putNumber("elevator_speed", elevator_encoder.getRaw());
      //SmartDashboard.putNumber("drive_encode_yaw", ypr[1]);


      //SmartDashboard.putString("intake left", l_intake.get().toString());
      
      SmartDashboard.putNumber("pigeon left", _pigeon.getPitch());
      //SmartDashboard.putString("intake right", r_intake.get().toString());
    }

  @Override
  public void teleopInit() {
    //pivot_encoder.setPosition(0);
    elevator_encoder.setPosition(0);
    intake_pivot.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    pivot_pos = pivot_encoder.getPosition();
    // Driving
    m_robotDrive.arcadeDrive(d_control.getLeftY() * m_drive_speed, d_control.getRightX() * m_turn_speed);


    // LED

    for(int i = 0; i < LED_buffer.getLength(); i++){
      LED_buffer.setHSV(i, (i+(int)additive)*3, 255, 255);
    }

    additive += 0.3;

    
    
    //LED.setData(LED_buffer);
    //for(int i = 0; i < LED_buffer.getLength(); i++){
    //  LED_buffer.setHSV(i, 165, 255, 255);
    //}


    intake_pivot.set(o_control.getRightY() * 0.3);
    e_motors.set(o_control.getElbowSpeed() * 0.2);

    if(d_control.getBButton()){

      switch (led_seq){
        case 1:
          for(int i = 0; i < LED_buffer.getLength(); i++){
            LED_buffer.setHSV(i, 120, 255, 255);
          }
          if(led_ms > 500){
            led_seq = 2;
            led_ms = 0;
          }

          break;
        case 2:
          for(int i = 0; i < LED_buffer.getLength(); i++){
            LED_buffer.setHSV(i, 0, 255, 255);
          }
          if(led_ms > 500){
            led_seq = 1;
            led_ms = 0;
          }
          break;
      }

      LED.setData(LED_buffer);

      led_ms+= 20;


    }
    

    //set to intaking position
    if(o_control.getAButton()){
      pivot_lerp.setTarget(-45);
      elevator_lerp.setTarget(-17);

      pivot_lerp.lerpPeriodic();
      elevator_lerp.lerpPeriodic();

      for(int i = 0; i < LED_buffer.getLength(); i++){
        LED_buffer.setHSV(i, 90, 255, 255);
      }

    }

    // TOP
    //Set to 0

    if (o_control.getYButtonPressed()) {
      tel_seq = 0;
    }
    if(o_control.getYButton()){

      for(int i = 0; i < LED_buffer.getLength(); i++){
        LED_buffer.setHSV(i, 30, 255, 255);
      }

      switch(tel_seq){

        case 0: // point intake up
          elevator_lerp.setTarget(-10);
          pivot_lerp.setTarget(-17);
          //elevator_lerp.setThresh(20);

          if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
          elevator_lerp.lerpPeriodic();
          pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 1;
          }

          break;

        case 1: // bring elevator up
          elevator_lerp.setTarget(-50);
          pivot_lerp.setTarget(-46);
          //elevator_lerp.setThresh(0.5);

          if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
            elevator_lerp.lerpPeriodic();
            pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 2;
          }

          break;

        case 2:
          for(int i = 0; i < LED_buffer.getLength(); i++){
            LED_buffer.setHSV(i, 60, 255, 255);
          }
          break;
          

        
      }
    }
//                      XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    if (o_control.getXButtonPressed()) {
      tel_seq = 0;
    }
    if(o_control.getXButton()){
      for(int i = 0; i < LED_buffer.getLength(); i++){
        LED_buffer.setHSV(i, 150, 255, 255);
      }

      switch(tel_seq){

        case 0: // point intake up
          elevator_lerp.setTarget(-10);
          pivot_lerp.setTarget(-17);
          //elevator_lerp.setThresh(20);

          if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
          elevator_lerp.lerpPeriodic();
          pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 1;
          }

          break;

        case 1: // bring elevator up
          elevator_lerp.setTarget(-52);
          pivot_lerp.setTarget(-57);
          //elevator_lerp.setThresh(0.5);

          if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
            elevator_lerp.lerpPeriodic();
            pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 2;
          }

        case 2:
          for(int i = 0; i < LED_buffer.getLength(); i++){
            LED_buffer.setHSV(i, 60, 255, 255);
          }
          break;

        
      }
    }
    


          // RESET
    if (o_control.getBButtonPressed()) {
      tel_seq = 0;
    }

    if(o_control.getBButton()){

      for(int i = 0; i < LED_buffer.getLength(); i++){
        LED_buffer.setHSV(i, 15, 255, 255);
      }

      switch(tel_seq){
        case 0: // point intake up
          elevator_lerp.setTarget(-5);
          pivot_lerp.setTarget(-17);

          if(!pivot_lerp.isFinished() && !elevator_lerp.isFinished()){
            pivot_lerp.lerpPeriodic();
            elevator_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 1;
          }

          break;

          case 1: // bring elevator down
          elevator_lerp.setTarget(-2);
          if(!elevator_lerp.isFinished()){
            elevator_lerp.lerpPeriodic();
          }
          else{
            tel_seq = -1;
          }

          case 2: // rotate pivot back to start
          pivot_lerp.setTarget(-10);
          if(!pivot_lerp.isFinished()){
            
            pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 3;
          }
          
          break;

      }

    }

    LED.setData(LED_buffer);

    //switch (o_control.getPOV()){
    //  case 90:
    //    drive_lerp_l.setTarget(0);
    //    drive_lerp_r.setTarget(0);
//
    //    drive_lerp_l.lerpPeriodic();
    //    drive_lerp_r.lerpPeriodic();
    //    break;
//
    //  case 180: // MODS
    //    drive_lerp_l.setTarget(90);
    //    drive_lerp_r.setTarget(-90);
//
    //    drive_lerp_l.lerpPeriodic();
    //    drive_lerp_r.lerpPeriodic();
    //    break;
    //}
    //if(o_control.getPOV() == 90){ // MODS
    //  drive_lerp_l.setTarget(0);
    //  drive_lerp_r.setTarget(0);
//
    //  drive_lerp_l.lerpPeriodic();
    //  drive_lerp_r.lerpPeriodic();
    //}

    //if(elevator_encoder.getPosition() >= elevator_pos_max && o_control.getElbowSpeed() >= 0){
    //  e_motors.set(0);
    //}
//
    //else if(elevator_encoder.getPosition() <= 0.5 && o_control.getElbowSpeed() <= 0){
    //  e_motors.set(0);
    //}
    //else{
      
    //}
    
    //MovePivot(o_control.getRightY());
    intake.set((o_control.getLeftTriggerAxis() * 1) + (o_control.getRightTriggerAxis() * -1));
    
  }


  public void autonomousInit(){
      current_timer = 0;
      //elbow_encoder.setPosition(0);
      seq = 20;
      elevator_encoder.setPosition(0);
      pivot_encoder.setPosition(0);
      intake_pivot.setInverted(true);

      //drive_encoder_l.setPosition(0);
      //drive_encoder_r.setPosition(0);



    balancer.balancerInit();
  }

  public void autonomousPeriodic(){

    /* 
    switch (seq){
      case 0: // deliver cube
        if (current_timer < 1000){
          intake.set(0.5);
        }
        else{
          intake.set(0);
          seq = 1;
        }
        break;

      case 1: // drive until it hits the charge station
        if (_pigeon.getPitch() > -10 && !finished_climb){
          m_robotDrive.arcadeDrive(-0.6, 0);
        }
        else{
          if(cnt > 3){
            finished_climb = true;
            m_robotDrive.arcadeDrive(-0.6, 0);
          }
          if(cnt > 35){
            seq = 2;
          }
          cnt++;
          
        }
        break;
      case 2: // Approach engaged position
        balancer.balancerPeriodic();
        //m_robotDrive.arcadeDrive(0, 0);
        break;
    }

     */
              //                       DELIVER, MOBILITY, AND BALANCE
                                
    switch (seq){
      case 0: // deliver cube
        if (current_timer < 1000 && current_timer > 500){
          intake.set(0.5);
        }
        else if(current_timer >= 1000){
          intake.set(0);
          current_timer = 0;
          seq = 2;
        }

        if(current_timer < 500){
          pivot_lerp.setTarget(-5);
          pivot_lerp.lerpPeriodic();
        }
        break;

      //case 1:
      //  if(current_timer < 1000){
      //    intake_pivot.set(0.2);
      //  }
      //  else{
      //    seq = 2
      //  current_timer = 0;
      //    intake_pivot.set(0);
      //  }

      //  break;
      //

      case 2: // drive until it hits the charge station
        if (current_timer < 2300){ // 2300 for mobility, 1500 for charge   station
          m_robotDrive.arcadeDrive(0.6, 0);
        }
        else{
          current_timer = 0;
          seq = 5; // 5 for stagnant, 3 for mobility auto // 4 for immediate balance
        }

        break;

      case 3: // drive until it hits the charge station
        if (current_timer < 2650){
          m_robotDrive.arcadeDrive(0.4, 0);
        }
        else{
          current_timer = 0;
          seq = 6;
        }
        break;

      case 4: // Approach engaged position
        balancer.balancerPeriodic();
        break;

      case 5:
        m_robotDrive.arcadeDrive(0, 0);
        break;

      case 6: // drive until it hits the charge station
        if (current_timer < 1500){
          m_robotDrive.arcadeDrive(-0.6, 0);
        }
        else{
          current_timer = 0;
          seq = 4;

        }
        break;

      case 10:
        if (current_timer < 250){
          m_robotDrive.arcadeDrive(0, 0);
        }
        else{
          current_timer = 0;
          seq = 6;
        }
        break;

        case 20: // point intake up
        elevator_lerp.setTarget(-10);
        pivot_lerp.setTarget(-17);

        if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
          elevator_lerp.lerpPeriodic();
          pivot_lerp.lerpPeriodic();
        }
        else{
          e_motors.set(0);
          intake_pivot.set(0);
          seq = 21;
        }

        break;

      case 21: // bring elevator up
        elevator_lerp.setTarget(-53); //full dist = -50
        pivot_lerp.setTarget(-46);
        //elevator_lerp.setThresh(0.5);

        if(!elevator_lerp.isFinished() && !pivot_lerp.isFinished()){
          elevator_lerp.lerpPeriodic();
          pivot_lerp.lerpPeriodic();
        }
        else{
          current_timer = 0;
          e_motors.set(0);
          intake_pivot.set(0);
          seq = 22;
        }

        break;

      case 22: //Outake gameobject
        if (current_timer < 500){
          intake.set(1);
        }
        else{
          intake.set(0);
          current_timer = 0;
          seq = 23;
        }

        break;

        //set to 0
        case 23: // point intake up
        elevator_lerp.setTarget(-10);
        pivot_lerp.setTarget(-17);

        if(!pivot_lerp.isFinished() && !elevator_lerp.isFinished()){
          pivot_lerp.lerpPeriodic();
          elevator_lerp.lerpPeriodic();
        }
        else{
          e_motors.set(0);
          intake_pivot.set(0);
          seq = 24;
        }
        break;

        case 24: // bring elevator down
        elevator_lerp.setTarget(-2);
        if(!elevator_lerp.isFinished()){
          elevator_lerp.lerpPeriodic();
        }
        else{
          e_motors.set(0);
          intake_pivot.set(0);
          seq = 25;
        }

        case 25: // rotate pivot back to start
        pivot_lerp.setTarget(-10);
        if(!pivot_lerp.isFinished()){
          
          pivot_lerp.lerpPeriodic();
        }
        else{
          e_motors.set(0);
          intake_pivot.set(0);
          current_timer = 0;
          seq = 2;
        }
        
        break;
      
    }

     
    
    //switch (seq){
      //case 0: // deliver cube
      //  if (current_timer < 1000){
      //    intake.set(1); // -1 = cone, 1 = cube outtake
      //  }
      //  else{
      //    seq = 1;
      //    intake.set(0);
      //  }
      //  break;

      //case 0:
      //  balancer.balancerPeriodic();
      //   break;
        
      //case 1: // drive for 2.5 seconds
      //  if (current_timer < 2500){
      //    m_robotDrive.arcadeDrive(0.7, 0);
      //  }
      //  else{
      //    seq = 2;
      //  }
      //  break;
      //case 2: // stop. STOP. STOOOOP (sponsored by KILLBOX™ )
      //  
      //  m_robotDrive.arcadeDrive(0, 0);
      //  break;
    
    


    current_timer += 20;
  }

  private void MovePivot(double input){
    if(input == 0){
      return;
    }
    if(input > 0 && pivot_pos >= pivot_limit_far){
      return;
    }
    if(input < 0 && pivot_pos <= pivot_limit_close){
      return;
    }
    if(pivot_pos+slow_distance >= pivot_limit_far || pivot_pos - slow_distance <= pivot_limit_close){
      coefficient = slow_co;
    }
    else{
      coefficient = def_co;
    }

    intake_pivot.set(input * coefficient);
    return;
  }


  public static double getRobotPitch(){
    return _pigeon.getPitch();
  }
  @Override
  public void testInit(){
    balancer.balancerInit();

    pivot_encoder.setPosition(0);
    elevator_encoder.setPosition(0);

    intake_pivot.setInverted(true);

    SmartDashboard.updateValues();



  }
  @Override
  public void testPeriodic(){
    SmartDashboard.putNumber("drive_left_speed", m_leftMotor.get());
      SmartDashboard.putNumber("drive_right_speed", m_rightMotor.get());

      SmartDashboard.putNumber("Intake_speed", intake.get());
      SmartDashboard.putNumber("pivot_speed", intake_pivot.get());

      SmartDashboard.putNumber("pivot position", pivot_encoder.getPosition());
      SmartDashboard.putNumber("elevator position", elevator_encoder.getPosition());

    intake_pivot.set(o_control.getRightY() * 0.3);
    e_motors.set(o_control.getElbowSpeed() * 0.2);

    //set to intaking position
    if(o_control.getAButton()){
      pivot_lerp.setTarget(-45);
      elevator_lerp.setTarget(-7);

      pivot_lerp.lerpPeriodic();
      elevator_lerp.lerpPeriodic();
    }

    //Set to 0
    if (o_control.getBButtonPressed()) {
      tel_seq = 0;
    }

    if(o_control.getBButton()){
      switch(tel_seq){
        case 0: // point intake up
          elevator_lerp.setTarget(-10);
          pivot_lerp.setTarget(-17);

          if(!pivot_lerp.isFinished() && elevator_lerp.isFinished()){
            pivot_lerp.lerpPeriodic();
            elevator_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 1;
          }

          break;

          case 1: // bring elevator down
          elevator_lerp.setTarget(-2);
          if(!elevator_lerp.isFinished()){
            elevator_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 2;
          }

          case 2: // rotate pivot back to start
          pivot_lerp.setTarget(-1);
          if(!pivot_lerp.isFinished()){
            
            pivot_lerp.lerpPeriodic();
          }
          else{
            tel_seq = 3;
          }
          
          break;
      }
    }
  }

  
}