/*
 * Team 4918, the Roboctopi, original creation on 11dec2019:
 * This is our 2019 MentorBot Robot code,
 * written by Jeff Gibbons and the Mentors.
 * (get their new album from Capitol Records today)!
 * Get 10 for only 1 cent!
 */

// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "frc/AnalogInput.h"
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
         } else if ( 7 < limea )  {         // if we're really close...
            autoDriveSpeed = 0.0;   //   go slow
         } else if ( 5 < limea ) {   // if we're a little farther...
            autoDriveSpeed = 0.1;   //   go a little faster
         } else if ( 2 < limea ) {   // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }
#else          
         if        ( 15 < limey ) {
            autoDriveSpeed = -0.1;
         } else if ( 12 < limey ) {   // if we're really close...
            autoDriveSpeed = 0.0;     //   go slow
         } else if (  8 < limey ) {   // if we're a little farther...
            autoDriveSpeed = 0.1;     //   go a little faster
         } else if (  2 < limey ) {   // if we're farther still...
            autoDriveSpeed = 0.15;    //   go a little faster still
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.20;    //   go as fast as we dare
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
   void RobotInit() {
                                // start a thread processing USB camera images
      std::thread visionThread(VisionThread);
      visionThread.detach();

      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);

          /* Configure Sensor Source for Primary PID */
      m_motorLSMaster.ConfigSelectedFeedbackSensor(
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 );

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
                                    // inverts encoder value positive/negative
      m_motorLSMaster.SetSensorPhase(true);
      // m_motorRSMaster.SetSensorPhase(false);

                       /* Set relevant frame periods to be at least as fast */
                       /* as the periodic rate.                             */
      m_motorLSMaster.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motorLSMaster.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

          /* Set the peak and nominal outputs */
      m_motorLSMaster.ConfigNominalOutputForward( 0, 10 );
      m_motorLSMaster.ConfigNominalOutputReverse( 0, 10 );
      m_motorLSMaster.ConfigPeakOutputForward(    1, 10 );
      m_motorLSMaster.ConfigPeakOutputReverse(   -1, 10 );

      m_motorLSMaster.ConfigPeakCurrentLimit(40);
      m_motorLSMaster.ConfigPeakCurrentDuration(100);
      m_motorLSMaster.ConfigContinuousCurrentLimit(40);
      m_motorLSMaster.EnableCurrentLimit(true);
      m_motorRSMaster.ConfigPeakCurrentLimit(40);
      m_motorRSMaster.ConfigPeakCurrentDuration(100);
      m_motorRSMaster.ConfigContinuousCurrentLimit(40);
      m_motorRSMaster.EnableCurrentLimit(true);

         /* Set Motion Magic gains in slot0 - see documentation */
      m_motorLSMaster.SelectProfileSlot( 0, 0 );
      m_motorLSMaster.Config_kF( 0, 0.3, 10 );
      m_motorLSMaster.Config_kP( 0, 5, 10 );
      m_motorLSMaster.Config_kI( 0, 0.0, 10 );
      m_motorLSMaster.Config_kD( 0, 0.0, 10 );

         /* Set acceleration and cruise velocity - see documentation */
      m_motorLSMaster.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motorLSMaster.ConfigMotionAcceleration(   1500, 10 );

      m_motorLSMaster.ConfigClosedloopRamp(0.0);
      m_motorLSMaster.ConfigOpenloopRamp(  0.0);
      m_motorRSMaster.ConfigClosedloopRamp(0.0);
      m_motorRSMaster.ConfigOpenloopRamp(  0.0);
   }

   void AutonomousInit() override {
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
   }

   void AutonomousPeriodic() override {

      GetAllVariables();
                                       /* This is an example of how to read */
                                       /* a button on the joystick.         */
      if ( ( m_stick.GetRawButton(3) ) &&       // If driver is pressing the
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
   }

   void TeleopInit() override {
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
                                                /* zeros the drive encoders */
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
   }

   void TeleopPeriodic() override {


      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.

#ifndef JAGNOTDEFINED
      {
         static int  iCallCount = 0;
         if ( 0 == iCallCount%1000000 )  {   // every 2 seconds
            cout << "joy: " << m_stick.GetX() << "/" << m_stick.GetY();
            cout << endl;
            cout << "LSMotor: vel/pos/%: ";
            cout << m_motorLSMaster.GetSelectedSensorVelocity() << "/";
            cout << m_motorLSMaster.GetSelectedSensorPosition() << "/";
            cout << m_motorLSMaster.GetMotorOutputPercent() << "% ";
            cout << m_motorLSMaster.GetOutputCurrent();
            cout << endl;
            cout << "RSMotor: vel/pos/%: ";
            cout << m_motorRSMaster.GetSelectedSensorVelocity() << "/";
            cout << m_motorRSMaster.GetSelectedSensorPosition() << "/";
            cout << m_motorRSMaster.GetMotorOutputPercent() << "% ";
            cout << m_motorRSMaster.GetOutputCurrent();
            cout << endl;
            // max free speed for MinCims is about 6200
         }
         iCallCount++;
      }
#endif


                                       /* This is an example of how to read */
                                       /* a button on the joystick.         */
      if ( ( m_stick.GetRawButton(3) ) &&       // If driver is pressing the
         ( 1  == limev )                 ) {    // "drivetotarget" button and
                                                // the limelight has a target,
         DriveToTarget();        // then autonomously drive towards the target
      } else if ( m_stick.GetRawButton(1) ) {
         /* button 1 is the trigger button (on the front of the joystick)   */
         /* If button 1 pressed, assume 775Pro motors instead of mimi-CIM,  */
         /* and drive the motors directly instead, with Percent Output.     */
         m_drive.StopMotor();
         m_motorLSMaster.Set( ControlMode::PercentOutput,
                              -m_stick.GetY() );
         m_motorRSMaster.Set( ControlMode::PercentOutput,
                              -m_stick.GetX() );
      } else { 
 
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
                                    /* See AutonomousInit() for examples of */
                                    /* other ways we could drive the robot. */
         m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
      }

   }
 private:

         // Note for future: Need to add a gyro here.
   WPI_TalonSRX m_motorRSMaster{   8 };   // Right side drive motor
   WPI_TalonSRX m_motorLSMaster{   0 };   // Left  side drive motor   
   WPI_VictorSPX m_motorRSSlave1{  1 };   // Right side slave motor
   WPI_VictorSPX m_motorLSSlave1{ 14 };   // Left  side slave motor
   int iAutoCount;
   float drivePowerFactor = 0.8;
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
   //frc::PowerDistributionPanel pdp{0};
    frc::AnalogInput DistSensor1{0};

    std::shared_ptr<NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

    bool aBooleanVariable = false;   // just an example of a boolean variable

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
