/*
 * Team 4918, the Roboctopi, original creation on 11dec2019:
 * This is our 2019 MentorBot Robot code,
 * written by Jeff Gibbons and the Mentors.
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
#include <iomanip>
#include <vector>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <sstream>

using std::cout;
using std::endl;
using std::setw;
using std::setfill;              // so we can use "setfill('0') in cout streams
using namespace cv;

class Robot : public frc::TimedRobot {
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
      bool   joyButton[12];
      bool   conButton[13];
   } sCurrState, sPrevState;

   struct sMotorState {
      double targetVelocity_UnitsPer100ms;
      double sensorVmax, sensorVmin;
   };

   struct sMotorState LSMotorState, RSMotorState, TSMotorState, BSMotorState;

   bool aBooleanVariable = false;    // just an example of a boolean variable
   int    iCallCount = 0;
   double dTimeOfLastCall = 0.0;

 public:

   static struct sPowercellOnVideo {
      bool SeenByCamera;
      int  X;
      int  Y;
      int  Radius;
   } powercellOnVideo;

 private:
                 /* limelight variables: x: offset from centerline,         */
                 /*                      y: offset from centerline,         */
                 /*                      a: area of target, % (0-100),      */
                 /*                      v: whether the data is valid,      */
                 /*                      s: skew or rotation, deg (-90-0).  */
   double limex, limey, limea, limev, limes;


      /*---------------------------------------------------------------------*/
      /* VisionThread()                                                      */
      /* This function executes as a separate thread, to take 640x480-pixel  */
      /* video frames from the USB video camera, change to grayscale,        */
      /* and send to the DriverStation. It is documented here:               */
      /* https://docs.wpilib.org/en/latest/docs/software/vision-processing/  */
      /*         introduction/using-the-cameraserver-on-the-roborio.html     */
      /* http://opencv-cpp.blogspot.com/2016/10/                             */
      /*        object-detection-and-tracking-color-separation.html          */
      /*---------------------------------------------------------------------*/
   static void VisionThread() {
      int iBiggestCircleIndex = -1;
      int iBiggestCircleRadius = -1;
      
      cs::UsbCamera camera =
                 frc::CameraServer::GetInstance()->StartAutomaticCapture();
         //camera.SetResolution( 640, 480 );
      camera.SetResolution( 160, 120 );

      powercellOnVideo.SeenByCamera = false;  // Make sure no other code thinks
      powercellOnVideo.X = 0;                 // we see a powercell until we
      powercellOnVideo.Y = 0;                 // actually see one!
      powercellOnVideo.Radius = -1;

      cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
      cs::CvSource outputStreamStd =
              frc::CameraServer::GetInstance()->PutVideo( "Gray", 160, 120 );
              //frc::CameraServer::GetInstance()->GetVideo();
      cv::Mat source;
      cv::Mat output;
      cv::Mat hsvImg;
      cv::Mat threshImg;

      std::vector<Vec3f> v3fCircles;      // 3-element vector of floats;
                                          // this will be the pass-by reference
                                          // output of HoughCircles()
         /* HUE for YELLOW is 21-30.                                         */
         /* Adjust Saturation and Value depending on the lighting condition  */
         /* of the environment as well as the surface of the object.         */
      int     lowH = 21;       // Set Hue
      int     highH = 56;      // (orig: 30)

      int     lowS = 60;        // Set Saturation (orig: 200)
      int     highS = 255;

      int     lowV = 0;         // Set Value (orig: 102)
      int     highV = 245;      // (orig: 225)

      while( true ) {
         int iFrameCount = 0;

         usleep( 1000 );                               // wait one millisecond
         if ( cvSink.GrabFrame(source) )
         {
            iFrameCount++;
            cvtColor( source, output, cv::COLOR_BGR2GRAY );
            cvtColor( source, hsvImg, cv::COLOR_BGR2HSV );
            cv::inRange( hsvImg, cv::Scalar(lowH, lowS, lowV),
                         cv::Scalar(highH, highS, highV), threshImg );
                                                                  //Blur Effect
            cv::GaussianBlur( threshImg, threshImg, cv::Size(9, 9), 0);
            cv::dilate( threshImg, threshImg, 0 );      // Dilate Filter Effect
            cv::erode( threshImg, threshImg, 0 );       // Erode  Filter Effect
                     // fill circles vector with all circles in processed image
                     // HoughCircles() is an algorithm for detecting circles  
            cv::HoughCircles( threshImg, v3fCircles, CV_HOUGH_GRADIENT,
                              2,          // dp: inverse accumulator resolution
                              threshImg.rows / 2,   // min dist between centers
                                                    // of the detected circles
                              100,          // param1 (edge detector threshold)
                              50,  // p2: increase this to reduce false circles
                              40,                      // minimum circle radius
                              240 );                   // maximum circle radius
                              // was: threshImg.rows / 4, 100, 50, 10, 800 );

            iBiggestCircleIndex = -1;     // init to an impossible index
            iBiggestCircleRadius = -1;    // init to an impossibly-small radius

                                                             // for each circle
            for ( unsigned int i = 0; i < v3fCircles.size(); i++ ) {

               if ( 0 == iFrameCount%300 ) {          // every 10 seconds or so
                                      // Log the x and y position of the center
                                      // point of circle, and the radius.
                  std::cout << "Ball position X = " << v3fCircles[i][0] <<
                               ",\tY = " << v3fCircles[i][1] <<
                               ",\tRadius = " << v3fCircles[i][2] <<
                               " (" << v3fCircles[i][0] - threshImg.cols / 2 <<
                               ", " << threshImg.rows / 2 - v3fCircles[i][1] <<
                               ")" << endl;
               }

                     // if a bigger circle has been found than any found before
               if ( iBiggestCircleRadius < (int)v3fCircles[i][2] ) {
                  iBiggestCircleIndex = i;
                  iBiggestCircleRadius = (int)v3fCircles[i][2];
               }
                        // draw small green circle at center of object detected
               cv::circle( output,                    // draw on original image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           3,                     // radius of circle in pixels
                           cv::Scalar(0, 255, 0),                 // draw green
                           CV_FILLED );                            // thickness

                                      // draw red circle around object detected 
               cv::circle( output,                    // draw on original image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           (int)v3fCircles[i][2], // radius of circle in pixels
                           cv::Scalar(0, 0, 255),                   // draw red
                           3 );                                    // thickness
            }

            if ( -1 < iBiggestCircleIndex ) {  // if at least one ball was seen
                   // Then save all the info so other code can drive toward it.
                            // Convert X and Y positions so 0,0 is at center of
                            // camera image, with X increasing to the right and
                            // Y increasing up.
               powercellOnVideo.X = (int)v3fCircles[iBiggestCircleIndex][0] -
                                                         (int)threshImg.cols/2;
               powercellOnVideo.Y = (int)threshImg.rows/2 -
                                       (int)v3fCircles[iBiggestCircleIndex][1];
               powercellOnVideo.Radius = iBiggestCircleRadius;
               powercellOnVideo.SeenByCamera = true;
            } else {
               powercellOnVideo.SeenByCamera = false;
            }

            outputStreamStd.PutFrame( output );
         }
      }
   }


      /*---------------------------------------------------------------------*/
      /* GetAllVariables()                                                   */
      /* Retrieves all variable values from sensors, encoders,               */
      /* the limelight, etc.  It should be called at the beginning of        */
      /* every 20-millisecond tick.  Doing it this way, rather than          */
      /* having each function retrieve the values it needs when it needs     */
      /* them, should minimize CANbus traffic and keep the robot CPU fast.   */
      /*---------------------------------------------------------------------*/
   void GetAllVariables()  {

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

      limev = limenttable->GetNumber("tv",0.0);  // valid
      limex = limenttable->GetNumber("tx",0.0);  // x position
      limea = limenttable->GetNumber("ta",0.0);  // area
      limey = limenttable->GetNumber("ty",0.0);  // y position
      limes = limenttable->GetNumber("ts",0.0);  // skew
   }


      /*---------------------------------------------------------------------*/
      /* DriveToTarget()                                                     */
      /* DriveToTarget() drives autonomously towards a vision target.        */
      /* It returns true if the limelight data is valid, false otherwise.    */
      /*---------------------------------------------------------------------*/
   bool DriveToTarget()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;

      if ( 1 == limev )  {                       // if limelight data is valid
         double autoDriveSpeed;
             // limea is the area of the target seen by the limelight camera
             // and is in percent (between 0 and 100) of the whole screen area.
             // limey is the height above the center of the field of view
             // Could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on limey.
         if        ( 15 < limey ) {
            autoDriveSpeed = -0.1;
         } else if ( 12 < limey )  { // if we're really close...
            autoDriveSpeed = 0.0;    //   stop (or 0.08 to go slow)
         } else if (  8 < limey ) {  // if we're a little farther...
            autoDriveSpeed = 0.1;    //   go a little faster
         } else if (  2 < limey ) {  // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                    // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }

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
         m_drive.CurvatureDrive( 0.0, 0, 0 );                // stop the robot
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << ":" << limes << "." << endl;
      }

      return returnVal;

   }  /* DriveToTarget() */


 public:

      /*---------------------------------------------------------------------*/
      /* joystickDisplay()                                                   */
      /* Display all the joystick values on the console log.                 */
      /*---------------------------------------------------------------------*/
   void joystickDisplay( void ) {
         cout << "joy: " << setw(8) << m_stick.GetY() << "/" <<
		            setw(8) << m_stick.GetX();
         cout << " (" << setw(8) << sCurrState.joyY << "/" <<
		         setw(8) << sCurrState.joyX << " )";
         cout << endl;
   }


      /*---------------------------------------------------------------------*/
      /* motorDisplay()                                                      */
      /* Display all the current motor values for a specified motor on the   */
      /* console log.                                                        */
      /*---------------------------------------------------------------------*/
   void motorDisplay( const char * cTitle,
                      WPI_TalonSRX & m_motor,
                      struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      cout << cTitle << " vel(min:max)tgt/% A (pos): ";
      cout << setw(5) << motorVelocity*600/4096;
      cout << "(" << setw(5) << sMState.sensorVmin*600/4096 << ":";
      cout <<        setw(5) << sMState.sensorVmax*600/4096 << ")";
      cout << setw(5) << sMState.targetVelocity_UnitsPer100ms*600/4096 << "/ ";
      cout << setw(3) << m_motor.GetMotorOutputPercent() << "% ";
      cout << setw(5) << m_motor.GetStatorCurrent() << "A ";
      cout << "(" << setw(10) << m_motor.GetSelectedSensorPosition() << ")";
      cout << endl;
      sMState.sensorVmin = sMState.sensorVmax = motorVelocity;
   }


      /*---------------------------------------------------------------------*/
      /* motorFindMinMaxVelocity()                                           */
      /* Keep a motor's Vmin and Vmax updated, for display on the            */
      /* console log.                                                        */
      /*---------------------------------------------------------------------*/
   void motorFindMinMaxVelocity( WPI_TalonSRX & m_motor,
                                 struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      if ( motorVelocity < sMState.sensorVmin ) {
         sMState.sensorVmin = motorVelocity;
      } else if ( sMState.sensorVmax < motorVelocity ) {
         sMState.sensorVmax = motorVelocity;
      }
   }


      /*---------------------------------------------------------------------*/
      /* RunDriveMotors()                                                    */
      /* RunDriveMotors() drives the robot.  It uses joystick and console    */
      /* inputs to determine what the robot should do, and then runs the     */
      /* m_motorLSMaster and m_motorRSMaster motors to make the robot do it. */
      /*---------------------------------------------------------------------*/
   bool RunDriveMotors( void ) {
      static int iCallCount = 0;
      iCallCount++;

                  /* If joystick button 2 pressed, use the joystick position */
                  /* to adjust some variables to specific speeds, so we can  */
                  /* set the drive motors to those speeds in later code.     */
      if ( ( 0 == iCallCount%100 )  &&
           sCurrState.joyButton[2]     ) {

         if ( 0.5 < m_stick.GetY() &&
              LSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            LSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetY() < -0.5 &&
                     -790.0 * 4096 / 600 <
                                  LSMotorState.targetVelocity_UnitsPer100ms ) {
            LSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }

         if ( 0.5 < m_stick.GetX() &&
              RSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            RSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetX() < -0.5 && 
                     -790.0 * 4096 / 600 <
                                  RSMotorState.targetVelocity_UnitsPer100ms ) {
            RSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
      } 

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );


      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         joystickDisplay();

         motorDisplay( "LS:", m_motorLSMaster, LSMotorState );
         motorDisplay( "RS:", m_motorRSMaster, RSMotorState );

         // max free speed for MinCims is about 6200
         cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         cout << RoborioAccel.GetY() << "/";
         cout << RoborioAccel.GetZ() << endl;
      }

                                       /* Button 1 is the trigger button on */
                                       /* the front of the joystick.        */
      if ( ( sCurrState.joyButton[1] ) &&       // If driver is pressing the
         ( 1  == limev )                 ) {    // "drivetotarget" button and
                                                // the limelight has a target,
         DriveToTarget();        // then autonomously drive towards the target

                                        /* Button 3 is the topmost center   */
                                        /* button on the back of joystick.  */
      } else if ( sCurrState.joyButton[3] ) {

                     /* If button 3 pressed, drive the motors separately    */
                     /* (rather than with ArcadeDrive) with Percent Output, */
                     /* using the variables that have been set with the     */
                     /* joystick while button 3 is pressed.                 */
         if ( !sPrevState.joyButton[3] ) {  // if button has just been pressed
            m_drive.StopMotor();
                                // Set current sensor positions to zero
            m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
            m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
         }
         // m_motorLSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetY() );
         // m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetX() );
                        /* Use MotionMagic to set the position of the drive */
                        /* wheels, rather than setting their velocity.      */
         m_motorLSMaster.Set( ControlMode::MotionMagic, sCurrState.joyY*4096 );
         m_motorRSMaster.Set( ControlMode::MotionMagic, sCurrState.joyX*4096 );

                                     /* Button 2 is the bottom-most center  */
                                     /* button on the back of the joystick. */
      } else if ( sCurrState.joyButton[2] ) {
         static int iButtonPressCallCount = 0;
         if ( !sPrevState.joyButton[2] ) {  // if button has just been pressed
            m_drive.StopMotor();
            iButtonPressCallCount = 0;
         }

         if ( iButtonPressCallCount < 50 ) {       // turn motors on gently...
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
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
         m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );

                  /* To drive the robot using the drive motor encoders,     */
                  /* specifying each motor separately, we must              */
                  /* convert the desired speed to units / 100ms,            */
                  /* because the velocity setpoint is in units/100ms.       */
                  /* For example, to convert 500 RPM to units / 100ms:      */
                  /* 4096 Units/Rev * 500 RPM / 600 100ms/min               */
                  /* So code to drive the robot at up to 500 rpm in either  */
                  /* direction could look like this:                        */
         // double leftMotorOutput, rightMotorOutput;
         // if ( 0.0 < sCurrState.joyY ) {
         //    if ( 0.0 < sCurrState.joyX ) {
         //       leftMotorOutput  = sCurrState.joyY - sCurrState.joyX;
         //       rightMotorOutput = std::max( sCurrState.joyY, sCurrState.joyX );
         //    } else {
         //       leftMotorOutput  = std::max( sCurrState.joyY, -sCurrState.joyX );
         //       rightMotorOutput = sCurrState.joyY + sCurrState.joyX;
         //    }
         //  } else {
         //    if ( 0.0 < sCurrState.joyX ) {
         //       leftMotorOutput  = -std::max( -sCurrState.joyY, sCurrState.joyX );
         //       rightMotorOutput = sCurrState.joyY + sCurrState.joyX;
         //    } else {
         //       leftMotorOutput  = sCurrState.joyY - sCurrState.joyX;
         //       rightMotorOutput = -std::max( -sCurrState.joyY, -sCurrState.joyX );
         //    }
         //  }
         // m_motorLSMaster.Set( ControlMode::Velocity, 
         //                      leftMotorOutput  * 500.0 * 4096 / 600 );
         // m_motorRSMaster.Set( ControlMode::Velocity, 
         //                      rightMotorOutput * 500.0 * 4096 / 600 );
      }
      return true;
   }      // RunDriveMotors()


         /*------------------------------------------------------------------*/
         /* RunShooter()                                                     */
         /* RunShooter() drives the 2 shooter motors.  It uses joystick and  */
         /* console inputs to determine what the shooter should do, and then */
         /* runs the m_motorTopShooter and m_motorBotShooter motors to make  */
         /* the shooter do it.                                               */
         /*------------------------------------------------------------------*/
   bool RunShooter( void ) {
      static int iCallCount = 0;
      iCallCount++;

                /* The following code uses the 4 "joystick"-type buttons     */
                /* on the console to select different speeds                 */
                /* (targetVelocities) of the top and bottom shooter motors.  */
      if (   ( 0.5 < sCurrState.conY ) &&     // if console "joystick" is
            !( 0.5 < sPrevState.conY ) &&     // newly-pressed upward
           TSMotorState.targetVelocity_UnitsPer100ms < 3000.0 * 4096 / 600 ) {
         TSMotorState.targetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conY < -0.5 ) &&  // downward
                  -3000.0 * 4096 / 600 <
                                  TSMotorState.targetVelocity_UnitsPer100ms ) {
         TSMotorState.targetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }

      if (  ( 0.5 < sCurrState.conX ) &&     // if console "joystick" is
           !( 0.5 < sPrevState.conX ) &&     // newly-pressed to right
           BSMotorState.targetVelocity_UnitsPer100ms < 5000.0 * 4096 / 600 ) {
         BSMotorState.targetVelocity_UnitsPer100ms += 100.0 * 4096 / 600;
      } else if (  ( sCurrState.conX < -0.5 ) &&  // else if newly-pressed
                  !( sPrevState.conX < -0.5 ) &&  // to the left
                  -5000.0 * 4096 / 600 <
                                  BSMotorState.targetVelocity_UnitsPer100ms ) {
         BSMotorState.targetVelocity_UnitsPer100ms -= 100.0 * 4096 / 600;
      }

      motorFindMinMaxVelocity( m_motorTopShooter, LSMotorState );
      motorFindMinMaxVelocity( m_motorBotShooter, BSMotorState );

      if ( 2 == iCallCount%100 )  {   // every 2 seconds
         motorDisplay( "TS:", m_motorTopShooter, TSMotorState );
         motorDisplay( "BS:", m_motorBotShooter, BSMotorState );
      }

      if ( sCurrState.conButton[9] ){    // second-from-leftmost missile switch
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( sCurrState.conButton[12] ) {       // leftmost missile switch
         static int iButtonPressCallCount = 0;

         if ( !sPrevState.conButton[12] ) {     // if switch was just turned on
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
      return true;
   }     // RunShooter()


      /*---------------------------------------------------------------------*/
      /* motorInit()                                                         */
      /* Setup the initial configuration of a motor.  These settings can be  */
      /* superseded after this function is called, for the needs of each     */
      /* specific motor.                                                     */
      /*---------------------------------------------------------------------*/
   void motorInit( WPI_TalonSRX & m_motor ) {

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
                                                   // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.ConfigForwardLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                     LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.ConfigReverseLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
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
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      // m_motor.SetInverted(false);  // invert direction of motor itself.

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

            /* Set limits to how much current will be sent through the motor */
      m_motor.ConfigPeakCurrentLimit(10);    // 60 works here for miniCIMs
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
                                             // works fine here, with 40 for
                                             // ConfigContinuousCurrentLimit(),
                                             // but we can reduce to 10, 1, 10
                                             // for safety while debugging
      m_motor.ConfigContinuousCurrentLimit(10);
      m_motor.EnableCurrentLimit(true);

                 /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15,   10 );
         m_motor.Config_kP( 0, 0.2,    10 );
         m_motor.Config_kI( 0, 0.0002, 10 );
         m_motor.Config_kD( 0, 10.0,   10 );
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

                /* Set acceleration and cruise velocity - see documentation */
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

               /* Set ramp rate (how fast motor accelerates or decelerates) */
      m_motor.ConfigClosedloopRamp(0.0);
      m_motor.ConfigOpenloopRamp(  0.0);
   }


      /*---------------------------------------------------------------------*/
      /* RobotInit()                                                         */
      /* This function is called once when the robot is powered up.          */
      /* It performs preliminary initialization of all hardware that will    */
      /* be used in Autonomous or Teleop modes.                              */
      /*---------------------------------------------------------------------*/
   void RobotInit() {
      static int iRobotInitCallCount = 0;

      iRobotInitCallCount++;

      if ( 1 == iRobotInitCallCount ) {
                                // start a thread processing USB camera images
         std::thread visionThread(VisionThread);
         visionThread.detach();
      }

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
                                                    // if encoder is connected
      if ( OK == m_motorLSMaster.ConfigSelectedFeedbackSensor(
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15,   10 );   // these work well for
         m_motorLSMaster.Config_kP( 0, 0.2,    10 );   // RPMs above ~200
         m_motorLSMaster.Config_kI( 0, 0.0002, 10 );
         m_motorLSMaster.Config_kD( 0, 10.0,   10 );
         cout << "LSMaster encoder is okay" << endl;
      } else {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorLSMaster.Config_kP( 0, 0.0,  10 );
         m_motorLSMaster.Config_kI( 0, 0.0,  10 );
         m_motorLSMaster.Config_kD( 0, 0.0,  10 );
         cout << "LSMaster encoder is DISCONNECTED" << endl;
      }

                                                    // if encoder is connected
      if ( OK == m_motorRSMaster.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15,   10 );   // these work well for
         m_motorRSMaster.Config_kP( 0, 0.2,    10 );   // RPMs above ~200
         m_motorRSMaster.Config_kI( 0, 0.0002, 10 );
         m_motorRSMaster.Config_kD( 0, 10.0,   10 );
         cout << "RSMaster encoder is okay" << endl;

      } else {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorRSMaster.Config_kP( 0, 0.0,  10 );
         m_motorRSMaster.Config_kI( 0, 0.0,  10 );
         m_motorRSMaster.Config_kD( 0, 0.0,  10 );
         cout << "RSMaster encoder is DISCONNECTED" << endl;
      }

                                                     // if encoder is connected
      if ( OK == m_motorTopShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01,    10 );
         m_motorTopShooter.Config_kP( 0, 0.08,    10 );
         m_motorTopShooter.Config_kI( 0, 0.00008, 10 );
         m_motorTopShooter.Config_kD( 0, 0.8,     10 );
      } else {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorTopShooter.Config_kP( 0, 0.0,  10 );
         m_motorTopShooter.Config_kI( 0, 0.0,  10 );
         m_motorTopShooter.Config_kD( 0, 0.0,  10 );
      }

                                                     // if encoder is connected
      if ( OK == m_motorBotShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01,    10 );
         m_motorBotShooter.Config_kP( 0, 0.08,    10 );
         m_motorBotShooter.Config_kI( 0, 0.00008, 10 );
         m_motorBotShooter.Config_kD( 0, 0.8,    10 );
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorBotShooter.Config_kP( 0, 0.0,  10 );
         m_motorBotShooter.Config_kI( 0, 0.0,  10 );
         m_motorBotShooter.Config_kD( 0, 0.0,  10 );
      }
      iCallCount++;
   }


      /*---------------------------------------------------------------------*/
      /* DisabledInit()                                                      */
      /* This function is called once when the robot is disabled.            */
      /*---------------------------------------------------------------------*/
   void DisabledInit() override {
   }


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
      cout << "shoot 3 balls" << endl;
      m_drive.StopMotor();
      iCallCount=0;
   }


      /*---------------------------------------------------------------------*/
      /* AutonomousPeriodic()                                                */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Autonomous mode.                                              */
      /*---------------------------------------------------------------------*/
   void AutonomousPeriodic() override {

      GetAllVariables();

      iCallCount++;

      m_drive.StopMotor();
      LSMotorState.targetVelocity_UnitsPer100ms = 0;        // Left Side drive
      RSMotorState.targetVelocity_UnitsPer100ms = 0;        // Right Side drive

      LSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 250.0 * 4096 / 600;

      if (iCallCount<90) {
         if ( sCurrState.conButton[10] ) {
            LSMotorState.targetVelocity_UnitsPer100ms = 100.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 100.0 * 4096 / 600;
         } else if ( sCurrState.conButton[11] ) {
            LSMotorState.targetVelocity_UnitsPer100ms = -100.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = -100.0 * 4096 / 600;
         } else { 
            iCallCount=90;
         }
      } else if ( iCallCount<180 ) {
         LSMotorState.targetVelocity_UnitsPer100ms = 100.0 * 4096 / 600;
         RSMotorState.targetVelocity_UnitsPer100ms =-100.0 * 4096 / 600;
      } else {
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );     // 0.15 normally
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

      if ( 0 == iCallCount%100 ) {
         motorDisplay( "LS:", m_motorLSMaster, LSMotorState );
         motorDisplay( "RS:", m_motorRSMaster, RSMotorState );
      }
   }


      /*---------------------------------------------------------------------*/
      /* TeleopInit()                                                        */
      /* This function is called once when the robot enters Teleop mode.     */
      /*---------------------------------------------------------------------*/
   void TeleopInit() override {
      RobotInit();
                                                    // zero the drive encoders
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
   }


      /*---------------------------------------------------------------------*/
      /* TeleopPeriodic()                                                    */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Teleop mode.                                                  */
      /*---------------------------------------------------------------------*/
   void TeleopPeriodic() override {

      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.

      RunDriveMotors();

      RunShooter();


      sPrevState = sCurrState;

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         cout << "TelPeriodic loop duration: ";
         cout << frc::GetTime() - dTimeOfLastCall << endl;
               // use frc:Timer::GetFPGATimestamp() instead?
      }

      iCallCount++;
   }
 
};


Robot::sPowercellOnVideo Robot::powercellOnVideo = { true, 0, 0, -1 };

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

