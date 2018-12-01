package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


     // Wait for Start()
     waitForStart();


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

    sleep(1000);

}  // This brace closes the while Loop


 } // this brace closes the runOpMode()



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





    private boolean motorsBusy() {

        /*  This method checks all motors to see if they are busy.
         *
         *   If all motors are, returns = TRUE
         *   If all motors are not, returns = FALSE
         */
        return robot.DriveLeftFront.isBusy() && robot.DriveLeftRear.isBusy() && robot.DriveRightFront.isBusy() && robot.DriveRightRear.isBusy();
    }


}