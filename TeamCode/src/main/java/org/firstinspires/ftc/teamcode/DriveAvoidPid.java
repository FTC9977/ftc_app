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

public class PIDController
{
    private double m_P;                                 // factor for "proportional" control
    private double m_I;                                 // factor for "integral" control
    private double m_D;                                 // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;   // |maximum output|
    private double m_minimumOutput = -1.0;  // |minimum output|
    private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
    private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;              //is the pid controller enabled
    private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;          //the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;

    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate()
    {
        int     sign = 1;

        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    else
                        m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;

            if (m_result < 0) sign = -1;    // Record sign of result.

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
            if (Math.abs(m_result) > m_maximumOutput)
                m_result = m_maximumOutput * sign;
            else if (Math.abs(m_result) < m_minimumOutput)
                m_result = m_minimumOutput * sign;
        }
    }

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d)
    {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP() {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI() {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD() {
        return m_D;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID()
    {
        calculate();
        return m_result;
    }

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID(double input)
    {
        setInput(input);
        return performPID();
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous) {
        m_continuous = continuous;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    public void setContinuous() {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    public void setInputRange(double minimumInput, double maximumInput)
    {
        m_minimumInput = Math.abs(minimumInput);
        m_maximumInput = Math.abs(maximumInput);
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = Math.abs(minimumOutput);
        m_maximumOutput = Math.abs(maximumOutput);
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint)
    {
        int     sign = 1;

        if (m_maximumInput > m_minimumInput)
        {
            if (setpoint < 0) sign = -1;

            if (Math.abs(setpoint) > m_maximumInput)
                m_setpoint = m_maximumInput * sign;
            else if (Math.abs(setpoint) < m_minimumInput)
                m_setpoint = m_minimumInput * sign;
            else
                m_setpoint = setpoint;
        }
        else
            m_setpoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Retruns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError() {
        return m_error;
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */
    public void setTolerance(double percent) {
        m_tolerance = percent;
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget()
    {
        return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
    }

    /**
     * Begin running the PIDController
     */
    public void enable() {
        m_enabled = true;
    }

    /**
     * Stop running the PIDController.
     */
    public void disable() {
        m_enabled = false;
    }

    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public void reset()
    {
        disable();
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
    }

    /**
     * Set the input value to be used by the next call to performPID().
     * @param input Input value to the PID calculation.
     */
    public void setInput(double input)
    {
        int     sign = 1;

        if (m_maximumInput > m_minimumInput)
        {
            if (input < 0) sign = -1;

            if (Math.abs(input) > m_maximumInput)
                m_input = m_maximumInput * sign;
            else if (Math.abs(input) < m_minimumInput)
                m_input = m_minimumInput * sign;
            else
                m_input = input;
        }
        else
            m_input = input;
    }



}
