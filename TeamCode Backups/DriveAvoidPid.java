package org.firstinspires.ftc.teamcode;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Drive Avoid PID", group="CS")
//@Disabled

public class DriveAvoidPid extends LinearOpMode {

    // Declare opMode Members

    RRHardwareMap robot = new RRHardwareMap();
    BNO055IMU   imu;
    Orientation     lastAngles = new Orientation();
    double          globalAngle, power = .30, correction;
    boolean         aButton, bButton, touched;
    PIDController   pidRotate, pidDrive;


    // Called when init button is pressed

    @Override


    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        robot.DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Get Initialize IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        // Retrieve and initialize the IMU.  We expect the IMU to be attached to an I2C port
        // configured to be a sensor of ytype AdaFruit IMU and named "imu"

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode",  "calibrating....");
        telemetry.update();

        //make sure the IMU GYRO is calibrated before continuing

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
             sleep(50);
             idle();

        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        //drive until end of period


        while (opModeIsActive()) {

            //Use Gyro to drive in a straight line
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            // may need to change this to match Mechanum Wheels correctly
            robot.DriveLeftRear.setPower(-power + correction);
            robot.DriveRightRear.setPower(-power);
            robot.DriveLeftFront.setPower(-power + correction);
            robot.DriveRightFront.setPower(-power);

            // We record the sensor values because we will test then in more than one place
            //  with time passing between those places.

            aButton = gamepad1.a;
            bButton = gamepad1.b;

            if (aButton || bButton) {

                // Backup
                robot.DriveLeftRear.setPower(power);
                robot.DriveRightRear.setPower(power);
                robot.DriveLeftFront.setPower(power);
                robot.DriveRightFront.setPower(power);

                sleep(500);

                // STOP
                robot.DriveLeftRear.setPower(0);
                robot.DriveRightRear.setPower(0);
                robot.DriveLeftFront.setPower(0);
                robot.DriveRightFront.setPower(0);


                // TURN 90 Degrees to RIGHT

                if (aButton) rotate(-90, power);

                // TURN 90 Degrees to LEFT

                if (bButton) rotate(90, power);
            }
        }

        //  TURN all MOTORS off:
        robot.DriveLeftRear.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveRightFront.setPower(0);

        }

    /**
     * Resets the cumulative angle tracking to zerip
     */

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset
     * @return Angle in degrees.    + = LEFT     - = RIGHT
     */

    private double getAngle() {

        // We experimentally determine the Z axis is the axis we want to use for heading angle
        // We have to process the angle becuase the IMU works in euler angles, so the Z axis is
        // returned as 0 to +180, OR  0 to -180 ,  rolling back to -179 to +179 when rotation passes
        // 180 degrees.  We detect this transition and track the total cumulative angle of rotation

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
     * See if we are moving in a straight line and if not, return power correction value
     * @return POWER adjustment,  + = adjust LEFT;     - = adjust RIGHT
     */

    private double checkDirection() {

        // The gain value detemines how sensitive the correction is to the direciton changes
        // you will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // No adjustment needed
        else
            correction = -angle;       // Reverse sign of angle for correction

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate LEFT or RIGHT the number of degrees.   Does not support turning more than 180 degrees.
     * @param degrees Degress to turn:     + = turn LEFT;    - = turn RIGHT
     */

    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // Restart IMU tracking

        resetAngle();

        //getAngle() returns +, when rotating counter clockwise (LEFT)       AND
        //getAngle() returns -, when rotation clockwise (RIGHT)


        if (degrees < 0) {
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0) {
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // Set Power to rotate

        robot.DriveLeftRear.setPower(leftPower);
        robot.DriveRightRear.setPower(rightPower);
        robot.DriveLeftFront.setPower(leftPower);
        robot.DriveRightFront.setPower(rightPower);

        // Rotate until turn is completed

        if (degrees <0) {

            // on RIGHT TURN, we have to get off zero first

            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // LEFT TURN
            while (opModeIsActive() && getAngle() < degrees) {}

        // Turn the Motors off
        robot.DriveLeftRear.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveRightFront.setPower(0);

        // Wait for the rotation to stop
        sleep(1000);

        // reset Angle tracking on new heading
        resetAngle();
    }
}






