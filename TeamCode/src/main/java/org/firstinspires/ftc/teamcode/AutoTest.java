package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// These Imports are for PID Controll
import org.firstinspires.ftc.teamcode.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import java.lang.annotation.ElementType;

@Autonomous(name="AutoTest", group ="Auto")

public class AutoTest extends LinearOpMode {

    // Declare opMode Members

    RRHardwareMap robot = new RRHardwareMap();
    ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    static final double COUNTS_PER_MOTOR_REV = 1120;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // This is > 1.0 if motors are geared up
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    DigitalChannel  digIn, digIn2;                   // Defines the limit switches on Lander Hook


    PIDController   pidRotate, pidDrive;                  // Defines PID Control Elements
    BNO055IMU       imu;                                  // Declare IMU
    Orientation     lastAngles = new Orientation();
    double          globalAngle, power = .30, correction;


 @Override
 public void runOpMode() throws InterruptedException {

     robot.init(hardwareMap);
     detector = new GoldAlignDetector();
     detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
     detector.useDefaults();

     //Optional Tuning Parameters
     detector.alignSize = 50;
     detector.alignPosOffset = 200;
     detector.enable();

     // Initalize IMU
     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

     parameters.mode            = BNO055IMU.SensorMode.IMU;
     parameters.angleUnit       = BNO055IMU.AngleUnit.DEGREES;
     parameters.accelUnit       = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
     parameters.loggingEnabled  = false;

     // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
     // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
     // and named "imu".
     imu = hardwareMap.get(BNO055IMU.class, "imu");

     imu.initialize(parameters);


     // Set PID proportional value to start reducing power at about 50 degrees of rotation.
     pidRotate = new PIDController(.005, 0, 0);

     // Set PID proportional value to produce non-zero correction value when robot veers off
     // straight line. P value controls how sensitive the correction is.
     pidDrive = new PIDController(.05, 0, 0);

    telemetry.addData("Mode", "calibrating IMU.....");
    telemetry.update();

     // make sure the imu gyro is calibrated before continuing.
     while (!isStopRequested() && !imu.isGyroCalibrated())
     {
         sleep(50);
         idle();
     }

     telemetry.addData("Mode", "waiting for start");
     telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
     telemetry.update();





     // Wait for Start()
     waitForStart();

     telemetry.addData("Mode", "running");
     telemetry.update();

     sleep(1000);           // USED for DEBUGGING ONLY. Remove before COMPETITION






     while (opModeIsActive()) {

         /* There are several movements the robot will take once it is lowered from the hook.
          *
          *   Performing Constant Gold Mineral Detection
          *
          *   1. Move forward to clear lander legs
          *   2. Rotate 180 Degrees to the right (assuming camera is located on the left side of the robot.
          *   3. Move forward "x" inches and stop;   still scanning until detector.getAligned() == TRUE;
          *   4. Slowly move backwards, continuing scan.
          *   5. Once detector.getAligned() == TRUE. STOP
          *   6. Strafe to left "x" inches to knock off gold mineral
          *   7. Stop, strafe right "x" inches back to original starting position
          *   8. Make a decision:
          *       a. Are we attempting the second Gold Mineral for our alliance partner?
          *           if so, proceed using "Safe Paths" to navigate the field using obstical avoidance to
          *           avoid running into other robot.
          *       b. If NOT, proceed using "Safe Paths" to navigate the filed to drop off team marker.
          *
          *   9. Drop off team marker.
          *   10. Make a decision:
          *     a. Based on where we started, do we proceed along the wall in a straing line from marker drop to own crater?
          *     b. proceed to oposite crater and park?
          */

     while( motorsBusy() && !isStopRequested()) {
         telemetry.addLine("First Movement from lander");
         telemetry.update();


     }
    // Put main AS commands here
        // LanderDown();                  // Call Method to lower Robot down from Lander

         telemetry.addData("Calling PIDDriveForward Method", "running");
         telemetry.update();
         PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power


         telemetry.addData("Done with Method", " stopping");
         telemetry.update();
    sleep(1000);

}  // This brace closes the while Loop


 } // this brace closes the runOpMode()


    private boolean motorsBusy() {

        /*  This method checks all motors to see if they are busy.
         *
         *   If all motors are, returns = TRUE
         *   If all motors are not, returns = FALSE
         */
        return robot.DriveLeftFront.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy();
    }

    public void LanderDown() {

     /*
      * This method will be used to perform the following tasks:
      *
      *  1. Lower the robot down to the floor
      *  2. Slide to the right, clearing the hook
      *  3. Lower the Linear Actuator back to a compact state
      *  4. Hand control back to main program
      */

     digIn = robot.limitswTop;
     digIn2 = robot.limitswBot;


         while ( !digIn.getState() == false) {        // While loop to power LANDERHOOK motor until limitswTop is pressed
             robot.LANDERHOOK.setPower(.50);         // Set power of motor to 1/2 pwr to start.  Adjust for competition
         }
         robot.LANDERHOOK.setPower(0);              // Once switch is pressed, we should now be on the ground. Stop motor.

          sleep (1000);                 // This is used for DEBUGGING purposes only, REMOVE BEFORE COMPETITION.



        // Assumption:   Robot is successfully sitting on ground.  Hook will still be lander Hook on Field Element
        //   Next step is to move robot lander hook out of the Field Element bracket by moving robot to the left (x) inches


        DriveForward(.25, 2);  //Drive Forward 2" inches so hook clears lander attachment

        // Collapse Lander hook

        while (!digIn2.getState() == false) {
            robot.LANDERHOOK.setPower(.50);         // Set power of motor to 1/2 pwr to start. Adjust for competition
        }
        robot.LANDERHOOK.setPower(0);               // Once limitswBot is pressed, Linear Actuator should now be in a collapsed state
        sleep(1000);                    // This is used for DEBUGGING purposes only, REMOVE BEFORE COMPETITION.

        // This ends the LanderDown() method.

    }





    public boolean isPressed() {
     // Check's if the current state of the input pin
        return digIn.getState();
    }


    // The following methods are for Mecanum Wheel Operations.
    // at a later point, we can clean this up and put into its own class.

    public void DriveForward(double speed, int distance) {

        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target
        robot.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy()
                && robot.DriveLeftFront.isBusy() && robot.DriveLeftRear.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("RR Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    } // This brace closes out DriveForward Method


    public void StrafeRight(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Strafe Right
        robot.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    } //This brace ends StrafeRight Method


    public void StrafeLeft(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Strafe Left
        robot.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) -InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }   // This Brace ends StrafeLeft Method

    public void DriveBackwards(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Drive Backwards
        robot.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) -InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    } //This brace ends Drive BackwardsMethod

    public void RotateLeft(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Rotate Left
        robot.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) -InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    } // This brace closes out RotateLeft

    public void RotateRight(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to RotateRight
        robot.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }  // This brace closes out RotateRight Method


    public void DiagonalLeft(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to DiagonalLeft
        robot.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) -InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);

        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(MoveSpeed);
            robot.DriveRightRear.setPower(0);
            robot.DriveLeftFront.setPower(0);
            robot.DriveLeftRear.setPower(MoveSpeed);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }   //This brace ends DiagonalLeft Method

    public void DiagonalRight(double speed, int distance) {
        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set RUN_TO_POSITION

        robot.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Diagonal Right
        robot.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);


        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot.DriveRightFront.setPower(0);
            robot.DriveRightRear.setPower(MoveSpeed);
            robot.DriveLeftFront.setPower(MoveSpeed);
            robot.DriveLeftRear.setPower(0);

            telemetry.addData("DriveRightFront Speed is: ", MoveSpeed);
            telemetry.addData("DriveRightRear Speed is: ", MoveSpeed);
            telemetry.addData("DriveLeftFront Speed is  ", MoveSpeed);
            telemetry.addData("DriveLeftRear Speed is  ", MoveSpeed);
            telemetry.update();
        }  // THis brace close out the while Loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }    // This brace ends DiagonaDriveLeftRearight Method



    public void PIDDriveForward(double speed, double angle, int distance) {

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         */

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();

        // Use PID with imu input to drive in a straight line.
        correction = pidDrive.performPID(getAngle());


        //Initialize Mecanum Wheel DC Motor Behavior
        robot.DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // Set Motor Power to 0
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftRear.setPower(0);

        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target
        robot.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot.DriveLeftFront.setTargetPosition((int) InchesMoving);
        robot.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot.DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy()
                && robot.DriveLeftFront.isBusy() && robot.DriveLeftRear.isBusy()) {


            //Set Motor Power
            robot.DriveRightFront.setPower(speed + correction);
            robot.DriveLeftFront.setPower(speed);
            robot.DriveRightRear.setPower(speed);
            robot.DriveLeftRear.setPower(speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }   // END OF PIDDriveForward




    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }  // END OF resetAngle

}