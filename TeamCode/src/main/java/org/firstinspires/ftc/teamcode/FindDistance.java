package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.calib3d.*;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.nio.FloatBuff

import java.io.File;




@TeleOp(name="Find Distance Example", group="DogeCV")

public class FindDistance extends OpMode
{

    @Override
    public void init() {
        telemetry.addData("Status", " FindDistance Example");

    }

    @Override
    public void init_loop() {
    }
    // Code to run ONCE when the driver hits PLAY
     static{ System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}

     String filePath = "users/eharwood/test.jpg";
     Mat newImage = Imgcodecs.imread(filePath);




    @Override
    public void start(){
    }

    @Override
    public void loop() {


    }
    // Code to run ONCE after the driver hits STOP

    @Override
    public void stop(){

    }
}

