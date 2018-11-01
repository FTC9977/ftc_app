package org.firstinspires.ftc.teamcode;

 import com.disnodeteam.dogecv.CameraViewDisplay;
 import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 @Autonomous (name="DoubleV1", group="Double Sample")


public class DoubleSampleV1 extends LinearOpMode {

    // Declar opMode Members

    private GoldAlignDetector detector;

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private Servo servo;

    @Override

    public void runOpMode () {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        // Note, that the strings used here as parameters
        //  to 'get' must correspond to the names assigned during the robot configuration

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();


        //Optional Tuning Parameters

        detector.alignSize = 50;
        detector.alignPosOffset = 200;

        detector.enable();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        //  AUTO START //

        /*
         * This main section of the Autonomous program that will execute a series of movements using motor encoders
         * to navigate the field safely.  Please note, this is an example, and not competition code.  There is no
         * obsticle avoidance (i.e other robots, etc..) routines that are used.
         *
         * Its main purpose is to detect both gold minerals at both positions for a single alliance side (see field layout).
         */

        moveEncoder(200, - 200, 0.4);  //  Input Format:  ticksLeft, ticksRight, Speed)
        while( motorsBusy() && !isStopRequested()) {

            /* Method Details
             *
             *  Using a while loop, perform the following checks:
             *
             *  1. Call motorsBusy() method, check to see if all motors are busy   AND (&&)
             *  2. Call !isStopRequested() method, check to see if "Has the stopping of the opMode been requested??"
             *      Note:  This method is contained in the LineerOpMode() class
             *  3. Update Driver Station with a STATUS
             */

            telemetry.addData("Status", "Turning");
            telemetry.update();
        }
        setAllMotors(0);                        //  Turn all motors off once tickCounts have been reached.


        moveEncoder(1000, 1000, 0.4);
        while (motorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }
        setAllMotors(0);

        moveEncoder(-720, 720, 0.3);
        while ( motorsBusy () && !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(-800, 800, .4);
        while ( motorsBusy ()&& !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }
        setAllMotors(0);

        // Declare a variable that records the encoders currnet Position.  This will be used later..
        //
        int tickRef  = leftFront.getCurrentPosition();
        moveEncoder(2500,2500, .2);

        while(detector.getAligned() == false && motorsBusy() && !isStopRequested()) {

            /* Loop details:
             *
             *  Execute moveEncoder() method, while:
             *          - detector.getAligned() = FALSE   (i.e not aligned with gold mineral)
             *    AND   - motorsBust() = TRUE
             *    AND   - !isStopRequested = FALSE
             */
             telemetry.addData("Aligned", detector.getXPosition());
             telemetry.update();
        }
         /*
          *   Once the while loop breaks out of its control, which will be due detector.getAligned = TRUE  (i.e. were lined up with gold detector)
          *   Then declare a variable ticksLeft = 2500 - (leftFront.getCurrentPosition() - tickRef)
          *
          */
        int ticksLeft = 2500 - ( leftFront.getCurrentPosition() - tickRef);
        setAllMotors(0);

        moveEncoder(500, -500, 0.3);
        while ( motorsBusy() && !isStopRequested()) {
             telemetry.addData("Status", "Turning");
             telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(-400, 400, .6);
        while (motorsBusy() && !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(-300, 300, .6);
        while (motorsBusy() && !isStopRequested()){
            telemetry.addData("Status" , "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(-500, 500, .4);
        while (motorsBusy() && !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(ticksLeft, ticksLeft, 0.4);
        while (motorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Scoring");
            telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(2000, 2000, .2);
        while(detector.getAligned() == false && motorsBusy() && !isStopRequested()) {
            telemetry.addData("Aligned", detector.getXPosition());
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(500, -500, 4);
        while (motorsBusy() && !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(1000, 1000, .9);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Scoring");
            telemetry.update();
        }

        setAllMotors(0);

        // set servo to release Team Marker
        // uncomment the following lines, if using a serveo
        // servo.setPostion(-.8);
        // servo.setPostion(.5);

        moveEncoder(700, -700, .4);
        while (motorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        moveEncoder(1000,1000, .2);
        while (motorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }
        setAllMotors(0);
        moveEncoder(4000, 4000, 1.0);  // run to crater
        while (motorsBusy() && !isStopRequested()) {
            telemetry.addData("Status", "Turning");
            telemetry.update();
        }

        setAllMotors(0);
        detector.disable();
    }

    private void moveEncoder(int ticksLeft, int ticksRight, double speed) {
        int lfPose = leftFront.getCurrentPosition() + ticksLeft;
        int lrPose = leftRear.getCurrentPosition() + ticksLeft;
        int rfPos = rightFront.getCurrentPosition() + ticksRight;
        int rrPos = rightRear.getCurrentPosition() + ticksRight;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFront.setTargetPosition(lfPose);
        leftRear.setTargetPosition(lrPose);
        rightFront.setTargetPosition(rfPos);
        rightRear.setTargetPosition(rrPos);

        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
    }

    private boolean motorsBusy() {

        /*  This method checks all motors to see if they are busy.
         *
         *   If all motors are, returns = TRUE
         *   If all motors are not, returns = FALSE
         */
        return leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy();
    }

    private void setAllMotors(double speed){
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
    }

    private void turnToAngle(double angle, double speed) {

    }
}
