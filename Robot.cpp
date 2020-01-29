/*
 * Team 4918, the Roboctopi, original creation on 11dec2019:
 * This is our 2019 MentorBot Robot code,
 * written by Jeff Gibbons and the Mentors.
 * (get their new album from Capitol Records today)!
 */

// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "frc/AnalogInput.h"
#include "frc/BuiltInAccelerometer.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"
#include "frc/Joystick.h"
#include "frc/Servo.h"
#include "frc/Solenoid.h"
#include "frc/TimedRobot.h"
#include "frc/Timer.h"
#include "frc/RobotDrive.h"
#include "frc/drive/DifferentialDrive.h"
#include "cameraserver/CameraServer.h"
#include "vision/VisionRunner.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>
#include <sstream>

using std::cout;
using std::endl;

class Robot : public frc::TimedRobot {
 private:

   static void VisionThread() {

         // This function executes as a separate thread, to take 640x480 pixel
         // video frames from the USB video camera, change to grayscale, and
         // send to the DriverStation. It is documented here:
         // https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/
         //         669166-using-the-cameraserver-on-the-roborio
      cs::UsbCamera camera =
                 frc::CameraServer::GetInstance()->StartAutomaticCapture();
         //camera.SetResolution( 640, 480 );
      camera.SetResolution( 160, 120 );
      cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
      cs::CvSource outputStreamStd =
              frc::CameraServer::GetInstance()->PutVideo( "Gray", 160, 120 );
              //frc::CameraServer::GetInstance()->GetVideo();
      cv::Mat source;
      cv::Mat output;
      while( true ) {
         usleep( 1000 );                               // wait one millisecond
         if ( cvSink.GrabFrame(source) )
         {
            cvtColor( source, output, cv::COLOR_BGR2GRAY );
            outputStreamStd.PutFrame( output );
         }
      }
   }
    
        // GetAllVariables() retrieves all variable values from sensors,
        // encoders, the limelight, etc.  It should be called at the beginning
        // of every 20-millisecond tick.  Doing it this way, rather than
        // having each function retrieve the values it needs when it needs
        // them, should minimize CANbus traffic and keep the robot CPU fast.
   void GetAllVariables()  {
      int iLoopCount = 0;

      dTimeOfLastCall = frc::GetTime();   // use frc::Timer::GetFPGATimestamp() instead?

      sCurrState.joyX = m_stick.GetX();
      sCurrState.joyY = m_stick.GetY();
      sCurrState.conX = m_console.GetX();
      sCurrState.conY = m_console.GetY();
      for ( iLoopCount=1; iLoopCount<=11; iLoopCount++ ) {
         sCurrState.joyButton[iLoopCount] = m_stick.GetRawButton(iLoopCount);
         sCurrState.conButton[iLoopCount] = m_console.GetRawButton(iLoopCount);
      }
      sCurrState.conButton[12] = m_console.GetRawButton(12);

      limev = limenttable->GetNumber("tv",0.0);
      limex = limenttable->GetNumber("tx",0.0);
      limea = limenttable->GetNumber("ta",0.0);
      limey = limenttable->GetNumber("ty",0.0);
      limes = limenttable->GetNumber("ts",0.0);
   }
        // DriveToTarget() drives autonomously towards a vision target.
        // It returns true if the limelight data is valid, false otherwise.
   bool DriveToTarget()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;

      if ( 1 == limev )  {                       // if limelight data is valid
         double autoDriveSpeed;
             // limea is the area of the target seen by the limelight camera
             // and is in percent (between 0 and 100) of the whole screen area.
#if 0        
         if (10 < limea ) {
            autoDriveSpeed = -0.1;
         }
          else if ( 7 < limea )  {         // if we're really close...
            autoDriveSpeed = 0.0;   //   go slow
         } else if ( 5 < limea ) {   // if we're a little farther...
            autoDriveSpeed = 0.1;   //   go a little faster
         } else if ( 2 < limea ) {   // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }
#else          
          if      (15 < limey ) {
            autoDriveSpeed = -0.1;
         }
          else if ( 12 < limey )  {         // if we're really close...
            autoDriveSpeed = 0.0;   //   go slow
         } else if ( 8 < limey ) {   // if we're a little farther...
            autoDriveSpeed = 0.1;   //   go a little faster
         } else if ( 2 < limey ) {   // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }
#endif         
                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

         // May have to add/subtract a constant from limex here, to account
         // for the offset of the camera away from the centerline of the robot.
         if ( aBooleanVariable ) {
            m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
         } else if ( 0 <= limex )  {
                             // if target to the right, turn towards the right
            m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
         } else if ( limex < 0 ) {
                               // if target to the left, turn towards the left
            m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
         } else {
            m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
         }

      } else {                    // else limelight data is not valid any more
         // should we continue forward here?
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << "." << endl;
      }

      return returnVal;

   }  /* DriveToTarget() */

 public:
   void motorInit( WPI_TalonSRX & m_motor ) {

         /* Setup initial configuration of a motor.  These settings can be  */
         /* superseded after this function is called, for the needs of each */
         /* specific motor.                                                 */

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(       // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                           LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                           LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.OverrideLimitSwitchesEnable(true);
      }

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
                                    // inverts encoder value positive/negative
      m_motor.SetSensorPhase(true);
      m_motor.SetSensorPhase(true);
      // m_motor.SetInverted(false);
      // m_motor.SetInverted(false);

                       /* Set relevant frame periods to be at least as fast */
                       /* as the periodic rate.                             */
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

          /* Set the peak and nominal outputs */
      m_motor.ConfigNominalOutputForward( 0, 10 );
      m_motor.ConfigNominalOutputReverse( 0, 10 );
      m_motor.ConfigPeakOutputForward(    1, 10 );
      m_motor.ConfigPeakOutputReverse(   -1, 10 );

      m_motor.ConfigPeakCurrentLimit(60);    // 60 works here for miniCIMs
      m_motor.ConfigPeakCurrentDuration(1000);  // 1000 milliseconds (for 60 Amps)
                                             // works fine here, with 40 for
                                             // ConfigContinuousCurrentLimit(),
                                             // but we can reduce to 10, 1, 10
                                             // for safety while debugging
      m_motor.ConfigContinuousCurrentLimit(40);
      m_motor.EnableCurrentLimit(true);

         /* Set Closed Loop PIDF gains in slot0 - see documentation */
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(   // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motor.Config_kP( 0, 0.2, 10 );   // 0.08 for shooter (775 at 5:1)
         m_motor.Config_kI( 0, 0.0002, 10 ); // 0.00008 for shooter (775 at 5:1)
         m_motor.Config_kD( 0, 10.0, 10 );    // 0.8 for shooter (775 at 5:1)
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

         /* Set acceleration and cruise velocity - see documentation */
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

      m_motor.ConfigClosedloopRamp(0.0);
      m_motor.ConfigOpenloopRamp(  0.0);
   }

   void RobotInit() {
                                // start a thread processing USB camera images
      std::thread visionThread(VisionThread);
      visionThread.detach();

      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);

      motorInit( m_motorLSMaster );
      motorInit( m_motorRSMaster );
      motorInit( m_motorTopShooter );
      motorInit( m_motorBotShooter );
                                    // invert encoder value positive/negative
                                    // and motor direction, for some motors.
      m_motorLSMaster.SetSensorPhase(true);
      // m_motorLSMaster.SetInverted(false);
      m_motorRSMaster.SetSensorPhase(true);
      // m_motorRSMaster.SetInverted(false);
      m_motorTopShooter.SetSensorPhase(false);
      // m_motorTopShooter.SetInverted(false);
      m_motorBotShooter.SetSensorPhase(false);
      // m_motorBotShooter.SetInverted(false);

            /* Set Closed Loop PIDF gains in slot0 - see documentation */
      if ( OK == m_motorLSMaster.ConfigSelectedFeedbackSensor(   // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorLSMaster.Config_kP( 0, 0.2, 10 );   // 0.08 for shooter (775 at 5:1)
         m_motorLSMaster.Config_kI( 0, 0.0002, 10 ); // 0.00008 for shooter (775 at 5:1)
         m_motorLSMaster.Config_kD( 0, 10.0, 10 );    // 0.8 for shooter (775 at 5:1)
      } else {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorLSMaster.Config_kP( 0, 0.0, 10 );
         m_motorLSMaster.Config_kI( 0, 0.0, 10 );
         m_motorLSMaster.Config_kD( 0, 0.0, 10 );
      }

      if ( OK == m_motorRSMaster.ConfigSelectedFeedbackSensor(   // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorRSMaster.Config_kP( 0, 0.2, 10 );   // 0.08 for shooter (775 at 5:1)
         m_motorRSMaster.Config_kI( 0, 0.0002, 10 ); // 0.00008 for shooter (775 at 5:1)
         m_motorRSMaster.Config_kD( 0, 10.0, 10 );    // 0.8 for shooter (775 at 5:1)
      } else {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorRSMaster.Config_kP( 0, 0.0, 10 );
         m_motorRSMaster.Config_kI( 0, 0.0, 10 );
         m_motorRSMaster.Config_kD( 0, 0.0, 10 );
      }

      if ( OK == m_motorTopShooter.ConfigSelectedFeedbackSensor(   // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorTopShooter.Config_kP( 0, 0.08, 10 );   // 0.08 for shooter (775 at 5:1)
         m_motorTopShooter.Config_kI( 0, 0.00008, 10 ); // 0.00008 for shooter (775 at 5:1)
         m_motorTopShooter.Config_kD( 0, 0.8, 10 );    // 0.8 for shooter (775 at 5:1)
      } else {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorTopShooter.Config_kP( 0, 0.0, 10 );
         m_motorTopShooter.Config_kI( 0, 0.0, 10 );
         m_motorTopShooter.Config_kD( 0, 0.0, 10 );
      }

      if ( OK == m_motorBotShooter.ConfigSelectedFeedbackSensor(   // if encoder is connected
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorBotShooter.Config_kP( 0, 0.08, 10 );   // 0.08 for shooter (775 at 5:1)
         m_motorBotShooter.Config_kI( 0, 0.00008, 10 ); // 0.00008 for shooter (775 at 5:1)
         m_motorBotShooter.Config_kD( 0, 0.8, 10 );    // 0.8 for shooter (775 at 5:1)
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 5:1)
         m_motorBotShooter.Config_kP( 0, 0.0, 10 );
         m_motorBotShooter.Config_kI( 0, 0.0, 10 );
         m_motorBotShooter.Config_kD( 0, 0.0, 10 );
      }
   }

   void DisabledInit() override {
   }

   void AutonomousInit() override {
      cout << "shoot 3 balls" << endl;
      m_drive.StopMotor();
      iCallCount=0;
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
   }

   void AutonomousPeriodic() override {

#ifdef JAG_NOTDEFINED
      GetAllVariables();
                                       /* This is an example of how to read */
                                       /* a button on the joystick.         */
      if ( ( sCurrState.joyButton[3] ) &&       // If driver is pressing the
         ( 1  == limev )                 ) {    // "drivetotarget" button and
                                                // the limelight has a target,
         DriveToTarget();        // then autonomously drive towards the target
      } else { 
         if ( m_stick.GetTrigger() ) {
            // m_shiftingSolenoid.Set(  true ); // hi gear
         } else {
            // m_shiftingSolenoid.Set( false ); // lo gear
         }
         // m_drive.CurvatureDrive( m_stick.GetY(), 0, 0 );
      } 

                                       /* This is an example of how to read */
                                       /* a button on the console.          */
      if ( m_console.GetRawButton(3) ) {
            // wantBrakeEngaged = false;
      } else if ( m_console.GetRawButton(4) ) {
            // wantBrakeEngaged = true;
      } 
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
      m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );

         /* We could drive the motors directly instead, with Percent Output. */
         //  m_motorLSMaster.Set( ControlMode::PercentOutput,
         //                       -m_console.GetY() * drivePowerFactor );
         //  m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                       -m_console.GetY() * drivePowerFactor );

         // Or we could drive the motors with the xbox controller, and 
         // using CurvatureDrive.
         //   m_drive.CurvatureDrive( GetSimY(),
         //           m_xbox.GetX( frc::GenericHID::kLeftHand ),
         //           m_xbox.GetBumperPressed( frc::GenericHID::kRightHand ) );
         #endif

         iCallCount++;
         static double LStargetVelocity_UnitsPer100ms = 0; // Left Side drive
         static double RStargetVelocity_UnitsPer100ms = 0; // Right Side drive

         LStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         RStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         if (iCallCount<50) {
            LStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
            RStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         } else if (iCallCount<100) {
            LStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
            RStargetVelocity_UnitsPer100ms = 150.0 * 4096 / 600;
         } else if (iCallCount<150) {
            LStargetVelocity_UnitsPer100ms = 150.0 * 4096 / 600;
            RStargetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         } else if (iCallCount<200) {
            LStargetVelocity_UnitsPer100ms = 150.0 * 4096 / 600;
            RStargetVelocity_UnitsPer100ms =-150.0 * 4096 / 600;
         } else {
            LStargetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
            RStargetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
         }

         m_motorLSMaster.Set( ControlMode::Velocity, 
                              LStargetVelocity_UnitsPer100ms);
         m_motorRSMaster.Set( ControlMode::Velocity, 
                              RStargetVelocity_UnitsPer100ms);
   }

   void TeleopInit() override {
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
                                                /* zeros the drive encoders */
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
   }

   void TeleopPeriodic() override {

      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.

      static double LStargetVelocity_UnitsPer100ms = 0; // Left Side drive
      static double RStargetVelocity_UnitsPer100ms = 0; // Right Side drive
      static double TStargetVelocity_UnitsPer100ms = 0; // Top Shooter
      static double BStargetVelocity_UnitsPer100ms = 0; // Bottom Shooter

      static double RSsensorVmax, RSsensorVmin;
      static double LSsensorVmax, LSsensorVmin;
      static double TSsensorVmax, TSsensorVmin;
      static double BSsensorVmax, BSsensorVmin;
      

      if ( m_motorLSMaster.GetSelectedSensorVelocity() < LSsensorVmin )
         LSsensorVmin = m_motorLSMaster.GetSelectedSensorVelocity();
      else if ( LSsensorVmax < m_motorLSMaster.GetSelectedSensorVelocity() )
         LSsensorVmax = m_motorLSMaster.GetSelectedSensorVelocity();
      if ( m_motorRSMaster.GetSelectedSensorVelocity() < RSsensorVmin )
         RSsensorVmin = m_motorRSMaster.GetSelectedSensorVelocity();
      else if ( RSsensorVmax < m_motorRSMaster.GetSelectedSensorVelocity() )
         RSsensorVmax = m_motorRSMaster.GetSelectedSensorVelocity();

      if ( m_motorTopShooter.GetSelectedSensorVelocity() < TSsensorVmin )
         TSsensorVmin = m_motorTopShooter.GetSelectedSensorVelocity();
      else if ( TSsensorVmax < m_motorTopShooter.GetSelectedSensorVelocity() )
         TSsensorVmax = m_motorTopShooter.GetSelectedSensorVelocity();
      if ( m_motorBotShooter.GetSelectedSensorVelocity() < BSsensorVmin )
         BSsensorVmin = m_motorBotShooter.GetSelectedSensorVelocity();
      else if ( BSsensorVmax < m_motorBotShooter.GetSelectedSensorVelocity() )
         BSsensorVmax = m_motorBotShooter.GetSelectedSensorVelocity();

      if ( ( 0 == iCallCount%100 )  &&
           sCurrState.joyButton[2]     ) {

         if ( 0.5 < m_stick.GetY() &&
              LStargetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            LStargetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetY() < -0.5 &&
                     -790.0 * 4096 / 600 < LStargetVelocity_UnitsPer100ms ) {
            LStargetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
         if ( 0.5 < m_stick.GetX() &&
              RStargetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            RStargetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetX() < -0.5 && 
                     -790.0 * 4096 / 600 < RStargetVelocity_UnitsPer100ms ) {
            RStargetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
      }
               /* The following code uses the 4 "joystick"-type buttons     */
               /* on the console to select different speeds                 */
               /* (targetVelocities) of the top and bottom shooter motors.  */
            /* jagtemp; The button numbers here may be wrong, and should be corrected. */
            /* jagtemp; also the max speeds (forwards and backwards) need to be        */
            /*          found and entered here (replacing the "5990" values).          */
      if (   ( 0.5 < sCurrState.conY ) &&     // if console "joystick" is
            !( 0.5 < sPrevState.conY ) &&     // newly-pressed upward
           TStargetVelocity_UnitsPer100ms < 3000.0 * 4096 / 600 ) {
         TStargetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conY < -0.5 ) &&  // downward
                  -3000.0 * 4096 / 600 < TStargetVelocity_UnitsPer100ms ) {
         TStargetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }
      if (  ( 0.5 < sCurrState.conX ) &&     // if console "joystick" is
           !( 0.5 < sPrevState.conX ) &&     // newly-pressed to right
           BStargetVelocity_UnitsPer100ms < 5000.0 * 4096 / 600 ) {
         BStargetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conX < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conX < -0.5 ) &&  // to the left
                  -5000.0 * 4096 / 600 < BStargetVelocity_UnitsPer100ms ) {
         BStargetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }

#ifndef JAGNOTDEFINED
      if ( 0 == iCallCount%100 )  {   // every 2 seconds
#ifdef JAGNOTDEFINED
         cout << "joy: " << m_stick.GetY() << "/" << m_stick.GetX();
         cout << " (" << sCurrState.joyY << "/" << sCurrState.joyX << " )";
         cout << endl;
         cout << "LSMotor: vel/min:max/tgt % A: ";
         cout << m_motorLSMaster.GetSelectedSensorVelocity()*600/4096 << "/";
         cout << LSsensorVmin*600/4096 << ":" << LSsensorVmax*600/4096 << "/";
         cout << LStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorLSMaster.GetSelectedSensorPosition() << "/";
         cout << m_motorLSMaster.GetMotorOutputPercent() << "% ";
         cout << m_motorLSMaster.GetStatorCurrent();
         cout << endl;
         cout << "RSMotor: vel/min:max/tgt % A: ";
         cout << m_motorRSMaster.GetSelectedSensorVelocity()*600/4096 << "/";
         cout << RSsensorVmin*600/4096 << ":" << RSsensorVmax*600/4096 << "/";
         cout << RStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorRSMaster.GetSelectedSensorPosition() << "/";
         cout << m_motorRSMaster.GetMotorOutputPercent() << "% ";
         cout << m_motorRSMaster.GetStatorCurrent();
         cout << endl;
         // max free speed for MinCims is about 6200
         cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         cout << RoborioAccel.GetY() << "/";
         cout << RoborioAccel.GetZ() << endl;
#endif
         cout << "TopShooter: vel/min:max/tgt % A: ";
         cout << m_motorTopShooter.GetSelectedSensorVelocity()*600/4096 << "/";
         cout << TSsensorVmin*600/4096 << ":" << TSsensorVmax*600/4096 << "/";
         cout << TStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorTopShooter.GetSelectedSensorPosition() << "/";
         cout << m_motorTopShooter.GetMotorOutputPercent() << "% ";
         cout << m_motorTopShooter.GetStatorCurrent();
         cout << endl;
         cout << "BotShooter: vel/min:max/tgt % A: ";
         cout << m_motorBotShooter.GetSelectedSensorVelocity()*600/4096 << "/";
         cout << BSsensorVmin*600/4096 << ":" << BSsensorVmax*600/4096 << "/";
         cout << BStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorBotShooter.GetSelectedSensorPosition() << "/";
         cout << m_motorBotShooter.GetMotorOutputPercent() << "% ";
         cout << m_motorBotShooter.GetStatorCurrent();
         cout << endl;

         LSsensorVmin = LSsensorVmax = m_motorLSMaster.GetSelectedSensorVelocity();
         RSsensorVmin = RSsensorVmax = m_motorRSMaster.GetSelectedSensorVelocity();
         TSsensorVmin = TSsensorVmax = m_motorTopShooter.GetSelectedSensorVelocity();
         BSsensorVmin = BSsensorVmax = m_motorBotShooter.GetSelectedSensorVelocity();
      }
#endif

                                       /* Button 1 is the trigger button on */
                                       /* the front of the joystick.        */
      if ( ( sCurrState.joyButton[1] ) &&       // If driver is pressing the
         ( 1  == limev )                 ) {    // "drivetotarget" button and
                                                // the limelight has a target,
         DriveToTarget();        // then autonomously drive towards the target

                                        /* Button 3 is the topmost center   */
                                        /* button on the back of joystick.  */
      } else if ( sCurrState.joyButton[3] ) {
         /* If button 3 pressed, assume 775Pro motors instead of mimi-CIM,  */
         /* and drive the motors directly instead, with Percent Output.     */
         if ( !sPrevState.joyButton[3] ) {  // if button has just been pressed
            m_drive.StopMotor();
                                // Set current sensor positions to zero
            m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
            m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
         }
         m_motorLSMaster.Set( ControlMode::MotionMagic, sCurrState.joyY*4096 );
         m_motorRSMaster.Set( ControlMode::MotionMagic, sCurrState.joyX*4096 );
         // m_motorLSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetY() );
         // m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetX() );
                                     /* Button 2 is the bottom-most center  */
                                     /* button on the back of the joystick. */
      } else if ( sCurrState.joyButton[2] ) {
         static int iButtonPressCallCount = 0;
         if ( !sPrevState.joyButton[2] ) {  // if button has just been pressed
            m_drive.StopMotor();
            iButtonPressCallCount = 0;
         }

         if ( iButtonPressCallCount < 50 ) {   // turn motors on gently...
        	   m_motorLSMaster.Set( ControlMode::Velocity,
                  m_motorLSMaster.GetSelectedSensorVelocity() +
                     0.2 * ( LStargetVelocity_UnitsPer100ms -
                             m_motorLSMaster.GetSelectedSensorVelocity() ) );
            m_motorRSMaster.Set( ControlMode::Velocity,
                  m_motorRSMaster.GetSelectedSensorVelocity() +
                     0.2 * ( RStargetVelocity_UnitsPer100ms -
                             m_motorRSMaster.GetSelectedSensorVelocity() ) );
         } else {
            m_motorLSMaster.Set( ControlMode::Velocity, 
                                   LStargetVelocity_UnitsPer100ms);
            m_motorRSMaster.Set( ControlMode::Velocity, 
                                   RStargetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;
      } else { 
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
                                    /* See AutonomousInit() for examples of */
                                    /* other ways we could drive the robot. */

         m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
      }

       	/* Speed mode */
			/* Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			// double LStargetVelocity_UnitsPer100ms = m_stick.GetY() * 500.0 * 4096 / 600;
         // double RStargetVelocity_UnitsPer100ms = m_stick.GetX() * 500.0 * 4096 / 600;
			/* 500 RPM in either direction */
         // LStargetVelocity_UnitsPer100ms = 0;
         // RStargetVelocity_UnitsPer100ms = 0;
      if ( sCurrState.conButton[9] ){    // second-from-leftmost missile switch
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TStargetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BStargetVelocity_UnitsPer100ms );
      } else if ( sCurrState.conButton[12] ) {  // leftmost missile switch
         static int iButtonPressCallCount = 0;

         if ( !sPrevState.conButton[12] ) {  // if switch was just turned on
            iButtonPressCallCount = 0;
         }
         if ( iButtonPressCallCount < 50 ) {

        	   m_motorTopShooter.Set( ControlMode::Velocity,
                  m_motorTopShooter.GetSelectedSensorVelocity() +
                     0.2 * ( TStargetVelocity_UnitsPer100ms -
                             m_motorTopShooter.GetSelectedSensorVelocity() ) );
            m_motorBotShooter.Set( ControlMode::Velocity,
                  m_motorBotShooter.GetSelectedSensorVelocity() +
                     0.2 * ( BStargetVelocity_UnitsPer100ms -
                             m_motorBotShooter.GetSelectedSensorVelocity() ) );
         } else {
            m_motorTopShooter.Set( ControlMode::Velocity, 
                                   TStargetVelocity_UnitsPer100ms);
            m_motorBotShooter.Set( ControlMode::Velocity, 
                                   BStargetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;
         
      } else {
                           // slow down slowly, over about 2 seconds
#ifndef JAGNOTDEFINED
         m_motorTopShooter.Set( ControlMode::Velocity,
                              0.8 * m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
                              0.8 * m_motorBotShooter.GetSelectedSensorVelocity() );
#else
         m_motorTopShooter.Set( ControlMode::Velocity, 0 );
         m_motorBotShooter.Set( ControlMode::Velocity, 0 );
#endif
      }
         // m_motorLSMaster.Set( ControlMode::,
         //                      -m_stick.GetY() );
         // m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetX() );

      sPrevState = sCurrState;

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         cout << "TelPeriodic loop duration: ";
         cout << frc::GetTime() - dTimeOfLastCall << endl;
               // use frc:Timer::GetFPGATimestamp() instead?
      }

      iCallCount++;
   }
 private:

         // Note for future: Need to add a gyro here.
   WPI_TalonSRX m_motorLSMaster{   0 };   // Left  side drive motor
   WPI_TalonSRX m_motorRSMaster{   8 };   // Right side drive motor   
   WPI_VictorSPX m_motorLSSlave1{ 14 };   // Left  side slave motor
   WPI_VictorSPX m_motorRSSlave1{  1 };   // Right side slave motor
   WPI_TalonSRX m_motorTopShooter{ 2 };   // top motor on shooter
   WPI_TalonSRX m_motorBotShooter{ 9 };   // bottom motor on shooter

   int iAutoCount;
   float drivePowerFactor = 0.8;
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
   //frc::PowerDistributionPanel pdp{0};
    frc::AnalogInput DistSensor1{0};
   frc::BuiltInAccelerometer RoborioAccel{};

    std::shared_ptr<NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

   struct sState {
      double joyX;
      double joyY;
      double conX;
      double conY;
      bool   joyButton[16];
      bool   conButton[16];
   } sCurrState, sPrevState;

    bool aBooleanVariable = false;   // just an example of a boolean variable
   int    iCallCount = 0;
   double dTimeOfLastCall = 0.0;
                 /* limelight variables: x: offset from centerline,         */
                 /*                      y: offset from centerline,         */
                 /*                      a: area of target, % (0-100),      */
                 /*                      v: whether the data is valid,      */
                 /*                      s: skew or rotation, deg (-90-0).  */
    double limex, limey, limea, limev, limes;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
