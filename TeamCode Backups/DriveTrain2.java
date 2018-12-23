package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.Arrays;

/**
 * Created by eric on 12/2/18.
 *
 * This code was adopted by referencing code from:
 * https://github.com/ethan-schaffer/Sample-FTC-Code/blob/master/ftc_app-master/ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumSamples/MecanumTeleOp.java
 *
 *
 */

@TeleOp(name="MecanumTeleOP", group="CS")
//@Disabled


public class DriveTrain2 extends LinearOpMode {
    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD = .05;
    private static final double SCALEDPOWER = 1;  // The emphasis is on the current controller reading (vs. current motor power) on the drive train


    // Declare opMode Members

    RRHardwareMap robot = new RRHardwareMap();


    @Override

    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        // Following are for Basic Mecanum Drive in Arcade Mode

        double leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, landerHook, depositArm, pivotArm = 0;
        double extenderArm = 0;


        waitForStart();
        while (opModeIsActive()) {

            double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_y : 0;
            double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_x : 0;
            double inputC = Math.abs(gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.right_stick_x : 0;

            arcadeMecanum(inputY, inputX, inputC, robot.DriveLeftFront, robot.DriveRightFront, robot.DriveLeftRear, robot.DriveRightRear);

        /*
         * Replace with New Rover Ruckus code for:
         *   1. Manipulating the Sweeper Ingest
         *   2. Extending/Retracking the extenderArm Mecahnism
         *   3. Manipulating the Pivot arm for Mineral Delivery
         *   4. Define controls for Gamepad2, which controls this section of code
         */

            // Insert New Rover Ruckus Code for Mineral Ingest/Delivery options HERE



            // END of Code segment for Mineral Ingest/Delivery Options

        }
    }

    // The following is the code to control the driveTrain in Arcade mode.
    //  Note:  This is for TeleOP Mode only.. Not intended to be used in Autonomous Mode.
    //
    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = SCALEDPOWER;

        leftFront.setPower(leftFrontVal * scaledPower + leftFront.getPower() * (1 - scaledPower));
        rightFront.setPower(rightFrontVal * scaledPower + rightFront.getPower() * (1 - scaledPower));
        leftBack.setPower(leftBackVal * scaledPower + leftBack.getPower() * (1 - scaledPower));
        rightBack.setPower(rightBackVal * scaledPower + rightBack.getPower() * (1 - scaledPower));
    }
}
