package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name="AutoPID", group="PID")

public class DriveAvoidPid extends LinearOpMode {

    // Declare opMode Members

    RRHardwareMap robot = new RRHardwareMap();
    ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1120;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // This is > 1.0 if motors are geared up
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;






    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException{

    robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

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

        telemetry.addData("Mode", "calibrating...");
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

     while (opModeIsActive()){

         // Put main AS commands here

         sleep(1000);

         // Set up parameters for driving in a straight line.
         pidDrive.setSetpoint(0);
         pidDrive.setOutputRange(0, power);
         pidDrive.setInputRange(-90, 90);
         pidDrive.enable();

         // drive until end of period.


     }  // This brace closes the while Loop


    } // this brace closes the runOpMode()


}


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

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

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
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.


        // 11/28/18 - eharwood
        //  - we needed to add the additional motor controls for mechanum wheels
        //

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.DriveLeftFront.setPower(-power);
                robot.DriveLeftRear.setPower(power)
                robot.DriveRightFront.setPower(power);
                robot.DriveLeftRear.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                // TBD:  Set controls for Right Turn
                robot.DriveLeftFront.setPower(power);
                robot.DriveLeftRear.setPower(-power);
                robot.DriveRightFront.setPower(-power);
                robot.DriveRightRear.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                // TBD:  Set controls for Left Turn
                robot.DriveLeftFront.setPower(power);
                robot.DriveLeftRear.setPower(-power);
                robot.DriveRightFront.setPower(power);
                robot.DriveRightRear.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.

        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);


        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }




/* PID controller courtesy of Peter Tischler, with modifications. */

