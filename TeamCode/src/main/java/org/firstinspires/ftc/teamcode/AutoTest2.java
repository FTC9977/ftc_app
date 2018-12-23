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

@Autonomous(name="AutoTest2", group ="Auto")


public class AutoTest2 extends LinearOpMode {

    // Declare opMode Members

    RRHardwareMap robot = new RRHardwareMap();
    ElapsedTime runtime = new ElapsedTime();

    PIDController   pidRotate, pidDrive;                  // Defines PID Control Elements
    BNO055IMU       imu;                                  // Declare IMU
    Orientation     lastAngles = new Orientation();
    double          globalAngle, power = .30, correction;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //Initialize Mecanum Wheel DC Motor Behavior
        robot.DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all Mecanum wheel motors  power to Zero.
        robot.DriveRightFront.setPower(0);
        robot.DriveRightRear.setPower(0);
        robot.DriveLeftFront.setPower(0);
        robot.DriveLeftRear.setPower(0);


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
        pidDrive = new PIDController(0.05, 0, .05);

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

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Drive until end of period

        while (opModeIsActive()) {

            correction = pidDrive.performPID(getAngle());


            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 Motor Power", power);
            telemetry.update();



            // set power levels.
            robot.DriveLeftFront.setPower(power);
            robot.DriveLeftRear.setPower(power);
            robot.DriveRightFront.setPower(power);
            robot.DriveRightRear.setPower(power);

            sleep(100000);

            // set power levels.
            robot.DriveLeftFront.setPower(0);
            robot.DriveLeftRear.setPower(0);
            robot.DriveRightFront.setPower(0);
            robot.DriveRightRear.setPower(0);


        }
            // Put main AS commands here


            // Use PID with imu input to drive in a straight line.

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
}




