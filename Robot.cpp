/*
 * Team 4918, the Roboctopi, original creation on 11dec2019:
 * This is our 2019 MentorBot Robot code,
 * written by Jeff Gibbons and the Mentors.
 * (get their new album from Capitol Records today)!
 */

//TODO Add explanitory comments for why each include is necessary
#include "ctre/Phoenix.h"     //Used for motor controllers and encoders
#include "frc/Joystick.h"     //Used to read the state of the Joystick
#include "frc/TimedRobot.h"   //Base class for all FRC based robot functions
#include "frc/drive/DifferentialDrive.h"  
#include "cameraserver/CameraServer.h"    
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>  //TODO Is there a more appropriate WPIlib sleep command?

using std::cout;
using std::endl;

class Robot : public frc::TimedRobot {
 private:

   //TODO Add explanitory text for the purpse of this struct 
   // and why is it declared here and not at the bottom with the others?
   struct sMotorState {
      double targetVelocity_UnitsPer100ms; 
      double sensorVmax;
      double sensorVmin;
   };

   static void VisionThread() {

      // This function executes as a separate thread, to take 640x480 pixel
      // video frames from the USB video camera, change to grayscale, and
      // send to the DriverStation. It is documented here:
      // https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/
      //         669166-using-the-cameraserver-on-the-roborio
      
      cs::UsbCamera camera =
                 frc::CameraServer::GetInstance()->StartAutomaticCapture();
      
      camera.SetResolution( 160, 120 );
      cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
      cs::CvSource outputStreamStd =
              frc::CameraServer::GetInstance()->PutVideo( "Gray", 160, 120 );

      //TODO What is this for?
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
    
      
   void GetAllVariables()  {
      // Retrieves all variable values from sensors,
      // encoders, the limelight, etc.  It should be called at the beginning
      // of every 20-millisecond tick.  Doing it this way, rather than
      // having each function retrieve the values it needs when it needs
      // them, should minimize CANbus traffic and keep the robot CPU fast.
      
      dTimeOfLastCall = frc::GetTime();   // use frc::Timer::GetFPGATimestamp() instead?

      sCurrState.joyX = m_stick.GetX();
      sCurrState.joyY = m_stick.GetY();
      sCurrState.conX = m_console.GetX();
      sCurrState.conY = m_console.GetY();

      for ( int iLoopCount=1; iLoopCount<=11; iLoopCount++ ) {
         sCurrState.joyButton[iLoopCount] = m_stick.GetRawButton(iLoopCount);
         sCurrState.conButton[iLoopCount] = m_console.GetRawButton(iLoopCount);
      }
      sCurrState.conButton[12] = m_console.GetRawButton(12);

      limev = limenttable->GetNumber("tv",0.0);
      limex = limenttable->GetNumber("tx",0.0);
      limea = limenttable->GetNumber("ta",0.0);
      limey = limenttable->GetNumber("ty",0.0);
      //limes = limenttable->GetNumber("ts",0.0);
   }
      
   bool DriveToTarget()  {
      // Drives autonomously towards a vision target.
      // It returns true if the limelight data is valid, false otherwise.

      bool returnVal = false;

      //TODO Why is this declared here?
      static int  iCallCount = 0;

      iCallCount++;

      if ( limev == 1 )  {    // if limelight data is valid
         returnVal = true;

         double autoDriveSpeed = 0;
         
         // limea is the area of the target seen by the limelight camera
         // and is in percent (between 0 and 100) of the whole screen area.
         // limey is the height above the center of the field of view
         //TODO Update the reasoning below.  0.0 appears to be stopped.
         
         if        ( 15 < limey ) {
            autoDriveSpeed = -0.1;
         } else if ( 12 < limey )  { // if we're really close...
            autoDriveSpeed = 0.0;    //   go slow
         } else if (  8 < limey ) {  // if we're a little farther...
            autoDriveSpeed = 0.1;    //   go a little faster
         } else if (  2 < limey ) {  // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                    // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }

         //TODO May want to modify autoDriveSpeed depending
         // on the distance from the target determined
         // by sonar transducers.

         // May have to add/subtract a constant from limex here, to account
         // for the offset of the camera away from the centerline of the robot.
         
         if ( 0 <= limex )  {
                             // if target to the right, turn towards the right
            m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
         } else if ( limex < 0 ) {
                               // if target to the left, turn towards the left
            m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
         } else {
            //TODO Given the condtions above this else is never used
            m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
         }
      }
      
      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << "." << endl;
      }

      return returnVal;

   }  

 public:

   void motorDisplay( const char * cTitle,
		      WPI_TalonSRX & m_motor,
		      struct sMotorState & sMState ) {
            
      //TODO Add Description
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      cout << cTitle << " vel(min:max)tgt/% A (pos): ";
      cout << motorVelocity*600/4096;
      cout << "(" << sMState.sensorVmin*600/4096 << ":";
      cout <<        sMState.sensorVmax*600/4096 << ")";
      cout << sMState.targetVelocity_UnitsPer100ms*600/4096 << "/ ";
      cout << m_motor.GetMotorOutputPercent() << "% ";
      cout << m_motor.GetStatorCurrent() << "A ";
      cout << "(" << m_motor.GetSelectedSensorPosition() << ")";
      cout << endl;
      sMState.sensorVmin = sMState.sensorVmax = motorVelocity;
   }

   void motorFindMinMaxVelocity( WPI_TalonSRX & m_motor,
		                 struct sMotorState & sMState ) {
      //TODO Add Description.  This appears to be a High/Low record.
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      if ( motorVelocity < sMState.sensorVmin ) {
         sMState.sensorVmin = motorVelocity;
      } else if ( sMState.sensorVmax < motorVelocity ) {
         sMState.sensorVmax = motorVelocity;
      }
   }

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

      //TODO Clarify comments
      /*
         * Configure Talon SRX Output and Sensor direction.
         * Invert Motor to have green LEDs when driving Talon Forward
         * ( Requesting Positive Output ),
         * Phase sensor to have positive increment when driving Talon Forward
         * (Green LED)
      */
      
      //TODO Should this be an optional argument?
      // inverts encoder value positive/negative
      m_motor.SetSensorPhase(true);
      // m_motor.SetInverted(false);

      // Set relevant frame periods to be at least as fast as the periodic rate.                             
      m_motor.SetStatusFramePeriod(
                        StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motor.SetStatusFramePeriod(
                        StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

      //TODO Add some indication of the range of allowable values.
      // Set the peak and nominal outputs
      m_motor.ConfigNominalOutputForward( 0, 10 );
      m_motor.ConfigNominalOutputReverse( 0, 10 );
      m_motor.ConfigPeakOutputForward(    1, 10 );
      m_motor.ConfigPeakOutputReverse(   -1, 10 );

      //TODO Should this be an optional argument.  Perhaps a select case for type?
      m_motor.ConfigPeakCurrentLimit(10);    // 60 works here for miniCIMs
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
                                             // works fine here, with 40 for
                                             // ConfigContinuousCurrentLimit(),
                                             // but we can reduce to 10, 1, 10
                                             // for safety while debugging
      m_motor.ConfigContinuousCurrentLimit(10);
      m_motor.EnableCurrentLimit(true);

      // Set Closed Loop PIDF gains in slot0 - see documentation
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

      // Set acceleration and cruise velocity - see documentation
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

      //TODO Add comment regarding ramps
      m_motor.ConfigClosedloopRamp(0.0);
      m_motor.ConfigOpenloopRamp(  0.0);
   }

   void RobotInit() {
      
      // start a thread processing USB camera images
      std::thread visionThread(VisionThread);
      visionThread.detach();

      //TODO Should this be done after motor setup?
      // also, at present these are not installed
      //m_motorLSSlave1.Follow(m_motorLSMaster);
      //m_motorRSSlave1.Follow(m_motorRSMaster);

      motorInit( m_motorLSMaster );
      motorInit( m_motorRSMaster );
      motorInit( m_motorTopShooter );
      motorInit( m_motorBotShooter );

      // invert encoder value positive/negative and motor direction, for some motors.
      m_motorLSMaster.SetSensorPhase(true);
      // m_motorLSMaster.SetInverted(false);
      m_motorRSMaster.SetSensorPhase(true);
      // m_motorRSMaster.SetInverted(false);
      m_motorTopShooter.SetSensorPhase(false);
      // m_motorTopShooter.SetInverted(false);
      m_motorBotShooter.SetSensorPhase(false);
      // m_motorBotShooter.SetInverted(false);

      // Set Closed Loop PIDF gains in slot0 - see documentation
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
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 3:1)
         m_motorBotShooter.Config_kP( 0, 0.08, 10 );   // 0.08 for shooter (775 at 3:1)
         m_motorBotShooter.Config_kI( 0, 0.00008, 10 ); // 0.00008 for shooter (775 at 3:1)
         m_motorBotShooter.Config_kD( 0, 0.8, 10 );    // 0.8 for shooter (775 at 3:1)
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // 0.01 for shooter (775 at 3:1)
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
      iCallCount = 0;

      //TODO Verify this is redundant with RobotInit
      //m_motorLSSlave1.Follow(m_motorLSMaster);
      //m_motorRSSlave1.Follow(m_motorRSMaster);
   }

   void AutonomousPeriodic() override {
      //TODO Comment intent of Autonomous Periodic.  Start with the 
      // pseudo code for what we want it to do.

      GetAllVariables();

      iCallCount++;

      m_drive.StopMotor();    //TODO Why?

      //TODO These 4 lines appear to be conflicting
      LSMotorState.targetVelocity_UnitsPer100ms = 0; // Left Side drive
      RSMotorState.targetVelocity_UnitsPer100ms = 0; // Right Side drive

      LSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;

      if (iCallCount<50) {
         LSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
      } else if (iCallCount<100) {
         LSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms = 150.0 * 4096 / 600;
      } else if (iCallCount<400) {
         LSMotorState.targetVelocity_UnitsPer100ms = 150.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
      } else if (iCallCount<1500) {
         //TODO Why are these different left vs right?
         m_motorLSMaster.Config_kF( 0, 0.90, 10 );   // 0.15 normally
         m_motorLSMaster.Config_kP( 0, 0.2, 10 );    // 0.2  normally
         m_motorRSMaster.Config_kF( 0, 1.20, 10 );
         m_motorRSMaster.Config_kP( 0, 0.2, 10 );
         LSMotorState.targetVelocity_UnitsPer100ms = 20.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms =-20.0 * 4096 / 600;
      } else if (iCallCount<2000) {
         LSMotorState.targetVelocity_UnitsPer100ms = -10.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms =  10.0 * 4096 / 600;
      } else {
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // 0.15 normally
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );
         LSMotorState.targetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
      }

      m_motorLSMaster.Set( ControlMode::Velocity, 
                           LSMotorState.targetVelocity_UnitsPer100ms);
      m_motorRSMaster.Set( ControlMode::Velocity, 
                           RSMotorState.targetVelocity_UnitsPer100ms);

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );

      //TODO This misses the initial transition from <50 to <100.  Is this intentional?
      if ( 0 == iCallCount%100 ) {
         motorDisplay( "LS:", m_motorLSMaster, LSMotorState );
         motorDisplay( "RS:", m_motorRSMaster, RSMotorState );
      }
   }

   void TeleopInit() override {
      //TODO verify this is redundant with RobotInit
      //m_motorLSSlave1.Follow(m_motorLSMaster);
      //m_motorRSSlave1.Follow(m_motorRSMaster);

      // Zero the drive encoders
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
   }

   void TeleopPeriodic() override {

      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.
      
      //TODO Should there be a motor stop here as in the autonomous?

      //TODO Can this be removed?
      //LSMotorState.targetVelocity_UnitsPer100ms = 0; // Left Side drive
      //RSMotorState.targetVelocity_UnitsPer100ms = 0; // Right Side drive
      //TSMotorState.targetVelocity_UnitsPer100ms = 0; // Top Shooter
      //BSMotorState.targetVelocity_UnitsPer100ms = 0; // Bottom Shooter

      //TODO What is the signifigance of the formula (N * 4096/600) and should
      // it be made into a function using some centrally defined constants 
      // along with its inverse (N * 600 / 4096) ?

      if ( ( 0 == iCallCount%100 ) && sCurrState.joyButton[2] ) {

         if ( 0.5 < m_stick.GetY() &&
              LSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            LSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetY() < -0.5 &&
                     -790.0 * 4096 / 600 < LSMotorState.targetVelocity_UnitsPer100ms ) {
            LSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }

         if ( 0.5 < m_stick.GetX() &&
              RSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            RSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetX() < -0.5 && 
                     -790.0 * 4096 / 600 < RSMotorState.targetVelocity_UnitsPer100ms ) {
            RSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
      }

      // The following code uses the 4 "joystick"-type buttons
      // on the console to select different speeds
      // (targetVelocities) of the top and bottom shooter motors.
      if (   ( 0.5 < sCurrState.conY ) &&     // if console "joystick" is
            !( 0.5 < sPrevState.conY ) &&     // newly-pressed upward
           TSMotorState.targetVelocity_UnitsPer100ms < 3000.0 * 4096 / 600 ) {
         TSMotorState.targetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conY < -0.5 ) &&  // downward
                  -3000.0 * 4096 / 600 < TSMotorState.targetVelocity_UnitsPer100ms ) {
         TSMotorState.targetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }

      if (  ( 0.5 < sCurrState.conX ) &&     // if console "joystick" is
           !( 0.5 < sPrevState.conX ) &&     // newly-pressed to right
           BSMotorState.targetVelocity_UnitsPer100ms < 5000.0 * 4096 / 600 ) {
         BSMotorState.targetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conX < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conX < -0.5 ) &&  // to the left
                  -5000.0 * 4096 / 600 < BSMotorState.targetVelocity_UnitsPer100ms ) {
         BSMotorState.targetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );
      motorFindMinMaxVelocity( m_motorTopShooter, LSMotorState );
      motorFindMinMaxVelocity( m_motorBotShooter, BSMotorState );


      //TODO Should this be a separate display function like motorDisplay


      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         cout << "joy: " << m_stick.GetY() << "/" << m_stick.GetX();
         cout << " (" << sCurrState.joyY << "/" << sCurrState.joyX << " )";
         cout << endl;
         cout << "LSMotor: vel/min:max/tgt % A: ";
         cout << m_motorLSMaster.GetSelectedSensorVelocity()*600/4096 << "/";
         //cout << LSsensorVmin*600/4096 << ":" << LSsensorVmax*600/4096 << "/";
         //cout << LStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorLSMaster.GetSelectedSensorPosition() << "/";
         cout << m_motorLSMaster.GetMotorOutputPercent() << "% ";
         cout << m_motorLSMaster.GetStatorCurrent();
         cout << endl;
         cout << "RSMotor: vel/min:max/tgt % A: ";
         cout << m_motorRSMaster.GetSelectedSensorVelocity()*600/4096 << "/";
         //cout << RSsensorVmin*600/4096 << ":" << RSsensorVmax*600/4096 << "/";
         //cout << RStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorRSMaster.GetSelectedSensorPosition() << "/";
         cout << m_motorRSMaster.GetMotorOutputPercent() << "% ";
         cout << m_motorRSMaster.GetStatorCurrent();
         cout << endl;
         // max free speed for MinCims is about 6200
         //cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         //cout << RoborioAccel.GetY() << "/";
         //cout << RoborioAccel.GetZ() << endl;
         cout << "TopShooter: vel/min:max/tgt % A: ";
         cout << m_motorTopShooter.GetSelectedSensorVelocity()*600/4096 << "/";
         // xx cout << TSsensorVmin*600/4096 << ":" << TSsensorVmax*600/4096 << "/";
         // xx cout << TStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorTopShooter.GetSelectedSensorPosition() << "/";
         cout << m_motorTopShooter.GetMotorOutputPercent() << "% ";
         cout << m_motorTopShooter.GetStatorCurrent();
         cout << endl;
         cout << "BotShooter: vel/min:max/tgt % A: ";
         cout << m_motorBotShooter.GetSelectedSensorVelocity()*600/4096 << "/";
         // xx cout << BSsensorVmin*600/4096 << ":" << BSsensorVmax*600/4096 << "/";
         // xx cout << BStargetVelocity_UnitsPer100ms*600/4096 << " ";
         // cout << m_motorBotShooter.GetSelectedSensorPosition() << "/";
         cout << m_motorBotShooter.GetMotorOutputPercent() << "% ";
         cout << m_motorBotShooter.GetStatorCurrent();
         cout << endl;
         // motorDisplay( "LS:", m_motorLSMaster,   LSMotorState );
         // motorDisplay( "RS:", m_motorRSMaster,   RSMotorState );
         motorDisplay( "TS:", m_motorTopShooter, TSMotorState );
         motorDisplay( "BS:", m_motorBotShooter, BSMotorState );

         //LSMotorState.sensorVmin = LSMotorState.sensorVmax = m_motorLSMaster.GetSelectedSensorVelocity();
         //RSMotorState.sensorVmin = RSMotorState.sensorVmax = m_motorRSMaster.GetSelectedSensorVelocity();
         //TSMotorState.sensorVmin = TSMotorState.sensorVmax = m_motorTopShooter.GetSelectedSensorVelocity();
         //BSMotorState.sensorVmin = BSMotorState.sensorVmax = m_motorBotShooter.GetSelectedSensorVelocity();
      }


      //Button 1 is the trigger button on the front of the joystick.
      if ( ( sCurrState.joyButton[1] ) &&       // If driver is pressing the
         ( 1  == limev )                 ) {    // "drivetotarget" button and
                                                // the limelight has a target,
         DriveToTarget();        // then autonomously drive towards the target

      //Button 3 is the topmost center button on the back of joystick.
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

      //Button 2 is the bottom-most center button on the back of the joystick.
      } else if ( sCurrState.joyButton[2] ) {
         static int iButtonPressCallCount = 0;
         if ( !sPrevState.joyButton[2] ) {  // if button has just been pressed
            m_drive.StopMotor();
            iButtonPressCallCount = 0;
         }

         if ( iButtonPressCallCount < 50 ) {   // turn motors on gently...
        	   m_motorLSMaster.Set( ControlMode::Velocity,
                  m_motorLSMaster.GetSelectedSensorVelocity() +
                     0.2 * ( LSMotorState.targetVelocity_UnitsPer100ms -
                             m_motorLSMaster.GetSelectedSensorVelocity() ) );
            m_motorRSMaster.Set( ControlMode::Velocity,
                  m_motorRSMaster.GetSelectedSensorVelocity() +
                     0.2 * ( RSMotorState.targetVelocity_UnitsPer100ms -
                             m_motorRSMaster.GetSelectedSensorVelocity() ) );
         } else {
            m_motorLSMaster.Set( ControlMode::Velocity, 
                                   LSMotorState.targetVelocity_UnitsPer100ms);
            m_motorRSMaster.Set( ControlMode::Velocity, 
                                   RSMotorState.targetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;

      } else { 
         //Drive the robot according to the commanded Y and X joystick position.
         //See AutonomousInit() for examples of other ways we could drive the robot.
         m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
      }

      /* Speed mode */
		/* Convert 500 RPM to units / 100ms.
		* 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
		* velocity setpoint is in units/100ms
		*/
		
      // double LStargetVelocity_UnitsPer100ms = m_stick.GetY() * 500.0 * 4096 / 600;
      // double RStargetVelocity_UnitsPer100ms = m_stick.GetX() * 500.0 * 4096 / 600;

	   // 500 RPM in either direction
      // LSMotorState.targetVelocity_UnitsPer100ms = 0;
      // RSMotorState.targetVelocity_UnitsPer100ms = 0;
      if ( sCurrState.conButton[9] ){    // second-from-leftmost missile switch
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( sCurrState.conButton[12] ) {  // leftmost missile switch
         static int iButtonPressCallCount = 0;

         if ( !sPrevState.conButton[12] ) {  // if switch was just turned on
            iButtonPressCallCount = 0;
         }

         if ( iButtonPressCallCount < 50 ) {

        	   m_motorTopShooter.Set( ControlMode::Velocity,
                  m_motorTopShooter.GetSelectedSensorVelocity() +
                     0.2 * ( TSMotorState.targetVelocity_UnitsPer100ms -
                             m_motorTopShooter.GetSelectedSensorVelocity() ) );
            m_motorBotShooter.Set( ControlMode::Velocity,
                  m_motorBotShooter.GetSelectedSensorVelocity() +
                     0.2 * ( BSMotorState.targetVelocity_UnitsPer100ms -
                             m_motorBotShooter.GetSelectedSensorVelocity() ) );
         } else {
            m_motorTopShooter.Set( ControlMode::Velocity, 
                                   TSMotorState.targetVelocity_UnitsPer100ms);
            m_motorBotShooter.Set( ControlMode::Velocity, 
                                   BSMotorState.targetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;
         
      } else {
         // slow down slowly, over about 2 seconds
         m_motorTopShooter.Set( ControlMode::Velocity,
                              0.8 * m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
                              0.8 * m_motorBotShooter.GetSelectedSensorVelocity() );
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

   //TODO Need to add a gyro here.
   WPI_TalonSRX m_motorLSMaster{   0 };   // Left  side drive motor  //TODO Not best practices for a Talon to be id 0
   WPI_TalonSRX m_motorRSMaster{   8 };   // Right side drive motor 
   
   //TODO Remove.  These are not installed.  Remove  
   //WPI_VictorSPX m_motorLSSlave1{ 14 };   // Left  side slave motor
   //WPI_VictorSPX m_motorRSSlave1{  1 };   // Right side slave motor
   
   WPI_TalonSRX m_motorTopShooter{ 2 };   // top motor on shooter
   WPI_TalonSRX m_motorBotShooter{ 9 };   // bottom motor on shooter

   
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
   //frc::PowerDistributionPanel pdp{0};
   //frc::AnalogInput DistSensor1{0};
   //frc::BuiltInAccelerometer RoborioAccel{};

   std::shared_ptr<NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

   //TODO Is it best practices to initialize these structs with initial values of 0?
   struct sState {
      double joyX;
      double joyY;
      double conX;
      double conY;
      bool   joyButton[16];   //TODO In code these appear to only require 12 (0 to 11)
      bool   conButton[16];   //TODO In code these appear to only require 13 (0 to 12)
   } sCurrState, sPrevState;

   //TODO Why is this struct declared at the top and not here?
   struct sMotorState LSMotorState, RSMotorState, TSMotorState, BSMotorState;

   //TODO This appears to serve no puropse
   bool aBooleanVariable = false;   // just an example of a boolean variable

   int    iCallCount = 0;
   double dTimeOfLastCall = 0.0;

   /* limelight variables: x: offset from centerline,         
   *                      y: offset from centerline,
   *                      a: area of target, % (0-100),
   *                      v: whether the data is valid,
   *                      s: skew or rotation, deg (-90-0).  */
   double limev = 0;
   double limex = 0; 
   double limey = 0;
   double limea = 0;
   //double limes = 0;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
