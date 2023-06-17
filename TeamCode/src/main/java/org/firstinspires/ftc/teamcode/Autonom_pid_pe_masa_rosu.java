
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.util.ColorMatchResult;
//import com.qualcomm.robotcore.util.ColorMatch;

@Autonomous
public class Autonom_pid_pe_masa_rosu extends LinearOpMode {
    //de fapt AutonomousA2
    private OpenCvWebcam webcam;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private double height, width;
    public double lastTime;
    DcMotorEx motorFR, motorFL, motorBR, motorBL;

    String varrez = "Dreapta";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    //merge si pentru F2 acest autonom
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;

    public DcMotorEx arm1;
    //public DcMotorEx arm2;
    public DcMotorEx slider1;
    public DcMotorEx slider2;
    Servo loader1;
    //Servo loader2;
    Servo balanganitor;
    //Servo costa;
    ColorSensor ungur;

    BHI260IMU imu;




    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    double last_pos_servo;
    boolean merge = false;
    boolean merge2 = false;
    FunctiiDeAutonom f = new FunctiiDeAutonom(true);
    MechShow a = new MechShow(true);

    @Override
    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");

        arm1     = (DcMotorEx) hardwareMap.dcMotor.get("arm1");
        //arm2     = (DcMotorEx) hardwareMap.dcMotor.get("arm2");
        slider1   = (DcMotorEx) hardwareMap.dcMotor.get("slider1");
        slider2 = (DcMotorEx) hardwareMap.dcMotor.get("slider2");

        //loader2 =  hardwareMap.servo.get("loader2");
        loader1 = hardwareMap.get(Servo.class, "loader1" );
        balanganitor = hardwareMap.get(Servo.class, "balanganitor");
        //costa = hardwareMap.servo.get("costa");

        ungur = hardwareMap.get(ColorSensor.class, "senzor_gheara");
        imu = hardwareMap.get(BHI260IMU.class, "imuMaFut");



        //arm2.setDirection(DcMotorEx.Direction.REVERSE);
        //motorBR.setDirection(DcMotorEx.Direction.REVERSE);
        //motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        slider2.setDirection(DcMotorEx.Direction.REVERSE);


        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BHI260IMU.Parameters parametrii;

        parametrii = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
        );

        imu.initialize(parametrii);


        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        while (!isStarted() && !isStopRequested()) {

            // Create an object to receive the IMU angles
//            YawPitchRollAngles robotOrientation;
//            robotOrientation = imu.getRobotYawPitchRollAngles();
//
//            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
//            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);


            try {


                width = pipeline.getRect().width;
                height = pipeline.getRect().height;
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("Rectangle Width:", width);
                telemetry.addData("Rectangle Height:", height);
                telemetry.addData("Rect`angle H/W:", height / width);
                telemetry.addData("pozitie slider", slider1.getCurrentPosition());
                telemetry.addData("pozitie arm1", arm1.getCurrentPosition());
                //telemetry.addData("pozitie arm2", arm2.getCurrentPosition());

                if (height / width < 5 && height/width > 2.7) {

                    telemetry.addData("Rect", "2");
                    varrez = "Mijloc";
                }
                else if (height / width < 1.8 && height/width >= 1.2) {
                    telemetry.addData("Rect", "1");
                    varrez = "Stanga";
                }
                else {
                    telemetry.addData("Rect", "3");
                    varrez = "Dreapta";
                }
                telemetry.update();
            }
            catch (Exception E){
                height = 1;
                width = 1000;
                telemetry.addData("Webcam error", "Please restart");
            }
        }

        loader1.setPosition(0);



        Autonom.start();
        muie.start();
        mafutinel.start();
        //muie.start();
        while(!isStopRequested()){

        }

    }


    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            kdf(200);
//            loader2.setPosition(0.9);
//            loader1.setPosition(0.5);
            //balanganitor.setPosition(0.2);
            loader1.setPosition(0.1);



            if(varrez=="Dreapta" && !isStopRequested()) {
                imu.resetYaw();
                //108% - 90+

                /*parcare
                Translatare(105, 0,0.4);
                kdf(200);
                Translatare(0, 165, 0.4);


                */


//                Burdu(-1360,0.65,arm1,0.2);
//                kdf(200);
//                anaconda(1400,1400,0.8,400);
//                anaconda(1,1,0.6,10);
//                while (true){
//                    telemetry.addData("slider poz1", slider1.getCurrentPosition());
//                    telemetry.addData("slider poz2", slider1.getCurrentPosition());
//                }



                //cod bun
                Translatare(0,-165,0.58);
                kdf(300);
                Rotire(64,0.3);
                //Rotire(100,0.4);
//                kdf(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                //balanganitor.setPosition(0.95);
//                kdf(200);
                Translatare(0,-30,0.3);
                kdf(200);
                anaconda(1284,1284,0.7,1400);
                kdf(200);
                anaconda(600,600,0.7,0);
//                kdf(200);
                loader1.setPosition(0.3);
                kdf(200);
                Burdu(-95,0.77,arm1,0.9);
                kdf(200);
                Rotire(107,0.3);
                kdf(200);
                Translatare(24,0,0.3);
                kdf(200);
                Translatare(0,70,0.6);
                kdf(200);
                anaconda(412,412,0.4,500);
////                while (ungur.red() < 300){
//////                //Translatare(0,7,1);
//////                //}
////                kdf(200);
                //filament(700);

                while (ungur.red() < 120 || lastTime + 4000 > System.currentTimeMillis() ){//|| ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
                    Translatare(0, 11, 0.9);
                }


                loader1.setPosition(0.1);
//                kdf(200);

                anaconda(900,900,0.7,100);
                merge = true;
////                Burdu(-60,0.77,arm1,0.2);
////                kdf(200);
                Translatare(0,-90,0.45);
//                kdf(200);
                Rotire(-114,0.3);
                filament(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                kdf(200);
                Translatare(0,-29,0.45);
//                kdf(200);
                anaconda(1256,1256,0.7,400);
                filament(100);
                anaconda(1,1,0.7,0);
//                kdf(200);
//                kdf(200);
                loader1.setPosition(0.35);
                Burdu(-95,0.77,arm1,0.9);
                filament(100);
                Rotire(-227,0.3);
                filament(100);

                //Translatare(10,0,0.4);
//                Translatare(0,95,0.5);
//                anaconda(300,300,0.8,200);
////                while (ungur.red() < 120 || ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
////                    Translatare(0, 15, 0.9);
////                }
//
//                loader1.setPosition(0.1);
//                Translatare(0,-80,0.5);
//                merge2 = true;
//                Rotire(110,0.4);
//                loader1.setPosition(0.35);
//                Burdu(-24,0.77,arm1,0.9);
//                Rotire(-100,0.5);
                Translatare(-10,0,0.5);
                Translatare(0,-87,0.4);
//                Rotire(20,0.3);
                //sfarsit cod bun



                //                kdf(200);
//                Burdu(-1380,0.65, arm1,0.95);
//                kdf(200);
//                Rotire(-70,0.3);
//                kdf(200);
//                Translatare(0,-10,0.3);
                //schumy(97,25,-70,0);



                //kdf(300);
//                anaconda(1296,0.65);
//                kdf(300);
//                loader1.setPosition(0.5);



            }

            //else
            else if(varrez == "Mijloc" && !isStopRequested()) {
                imu.resetYaw();
                //108% - 90+

                /*parcare
                Translatare(105, 0,0.4);
                kdf(200);
                Translatare(0, 165, 0.4);


                */


//                Burdu(-1360,0.65,arm1,0.2);
//                kdf(200);
//                anaconda(1400,1400,0.8,400);
//                anaconda(1,1,0.6,10);
//                while (true){
//                    telemetry.addData("slider poz1", slider1.getCurrentPosition());
//                    telemetry.addData("slider poz2", slider1.getCurrentPosition());
//                }



                //cod bun
                Translatare(0,-165,0.58);
                kdf(300);
                Rotire(64,0.3);
                //Rotire(100,0.4);
//                kdf(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                //balanganitor.setPosition(0.95);
//                kdf(200);
                Translatare(0,-30,0.3);
                kdf(200);
                anaconda(1284,1284,0.7,1400);
                kdf(200);
                anaconda(600,600,0.7,0);
//                kdf(200);
                loader1.setPosition(0.3);
                kdf(200);
                Burdu(-95,0.77,arm1,0.9);
                kdf(200);
                Rotire(107,0.3);
                kdf(200);
                Translatare(24,0,0.3);
                kdf(200);
                Translatare(0,70,0.6);
                kdf(200);
                anaconda(412,412,0.4,500);
////                while (ungur.red() < 300){
//////                //Translatare(0,7,1);
//////                //}
////                kdf(200);
                //filament(700);

                while (ungur.red() < 120 || lastTime + 4000 > System.currentTimeMillis() ){//|| ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
                    Translatare(0, 11, 0.9);
                }


                loader1.setPosition(0.1);
//                kdf(200);

                anaconda(900,900,0.7,100);
                merge = true;
////                Burdu(-60,0.77,arm1,0.2);
////                kdf(200);
                Translatare(0,-90,0.45);
//                kdf(200);
                Rotire(-114,0.3);
                filament(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                kdf(200);
                Translatare(0,-29,0.45);
//                kdf(200);
                anaconda(1256,1256,0.7,400);
                filament(100);
                anaconda(1,1,0.7,0);
//                kdf(200);
//                kdf(200);
                loader1.setPosition(0.35);
                Burdu(-95,0.77,arm1,0.9);
                filament(100);
                Rotire(-227,0.3);
                filament(100);

                //Translatare(10,0,0.4);
//                Translatare(0,95,0.5);
//                anaconda(300,300,0.8,200);
////                while (ungur.red() < 120 || ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
////                    Translatare(0, 15, 0.9);
////                }
//
//                loader1.setPosition(0.1);
//                Translatare(0,-80,0.5);
//                merge2 = true;
//                Rotire(110,0.4);
//                loader1.setPosition(0.35);
//                Burdu(-24,0.77,arm1,0.9);
//                Rotire(-100,0.5);
                Translatare(-10,0,0.5);
//                Rotire(20,0.3);
                //sfarsit cod bun



                //                kdf(200);
//                Burdu(-1380,0.65, arm1,0.95);
//                kdf(200);
//                Rotire(-70,0.3);
//                kdf(200);
//                Translatare(0,-10,0.3);
                //schumy(97,25,-70,0);



                //kdf(300);
//                anaconda(1296,0.65);
//                kdf(300);
//                loader1.setPosition(0.5);


            }

            else if(varrez == "Stanga"  && !isStopRequested()) {
                imu.resetYaw();
                //108% - 90+

                /*parcare
                Translatare(105, 0,0.4);
                kdf(200);
                Translatare(0, 165, 0.4);


                */


//                Burdu(-1360,0.65,arm1,0.2);
//                kdf(200);
//                anaconda(1400,1400,0.8,400);
//                anaconda(1,1,0.6,10);
//                while (true){
//                    telemetry.addData("slider poz1", slider1.getCurrentPosition());
//                    telemetry.addData("slider poz2", slider1.getCurrentPosition());
//                }



                //cod bun
                Translatare(0,-165,0.58);
                kdf(300);
                Rotire(64,0.3);
                //Rotire(100,0.4);
//                kdf(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                //balanganitor.setPosition(0.95);
//                kdf(200);
                Translatare(0,-30,0.3);
                kdf(200);
                anaconda(1284,1284,0.7,1400);
                kdf(200);
                anaconda(600,600,0.7,0);
//                kdf(200);
                loader1.setPosition(0.3);
                kdf(200);
                Burdu(-95,0.77,arm1,0.9);
                kdf(200);
                Rotire(107,0.3);
                kdf(200);
                Translatare(24,0,0.3);
                kdf(200);
                Translatare(0,70,0.6);
                kdf(200);
                anaconda(412,412,0.4,500);
////                while (ungur.red() < 300){
//////                //Translatare(0,7,1);
//////                //}
////                kdf(200);
                //filament(700);
                lastTime = System.currentTimeMillis();
                while (ungur.red() < 120 || lastTime + 3000 > System.currentTimeMillis() ){//|| ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
                    Translatare(0, 11, 0.9);
                }


                loader1.setPosition(0.1);
//                kdf(200);

                anaconda(900,900,0.7,100);
                merge = true;
////                Burdu(-60,0.77,arm1,0.2);
////                kdf(200);
                Translatare(0,-90,0.45);
//                kdf(200);
                Rotire(-114,0.3);
                filament(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                kdf(200);
                Translatare(0,-29,0.45);
//                kdf(200);
                anaconda(1256,1256,0.7,400);
                filament(100);
                anaconda(1,1,0.7,0);
//                kdf(200);
//                kdf(200);
                loader1.setPosition(0.35);
                Burdu(-95,0.77,arm1,0.9);
                filament(100);
                Rotire(-227,0.3);
                filament(100);

                //Translatare(10,0,0.4);
//                Translatare(0,95,0.5);
//                anaconda(300,300,0.8,200);
////                while (ungur.red() < 120 || ((DistanceSensor) ungur).getDistance(DistanceUnit.MM) < 10 ) {
////                    Translatare(0, 15, 0.9);
////                }
//
//                loader1.setPosition(0.1);
//                Translatare(0,-80,0.5);
//                merge2 = true;
//                Rotire(110,0.4);
//                loader1.setPosition(0.35);
//                Burdu(-24,0.77,arm1,0.9);
//                Rotire(-100,0.5);
                Translatare(-10,0,0.5);
                Translatare(0,87,0.4);
//                Rotire(20,0.3);
                //sfarsit cod bun



                //                kdf(200);
//                Burdu(-1380,0.65, arm1,0.95);
//                kdf(200);
//                Rotire(-70,0.3);
//                kdf(200);
//                Translatare(0,-10,0.3);
                //schumy(97,25,-70,0);



                //kdf(300);
//                anaconda(1296,0.65);
//                kdf(300);
//                loader1.setPosition(0.5);



            }

        }
    });

    public Thread mafutinel = new Thread(new Runnable() {
        @Override
        public void run() {
            //filament(20);
            Messi(-1432,0.6, arm1,0.2);


        }
    });

    public  Thread muie = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()){
                if (merge){
                    filament(260);
                    Messi(-1445,0.7, arm1,0.2);
                    merge = false;
                }


                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                if (ungur instanceof DistanceSensor){
                    telemetry.addData("DiStAnTa", ((DistanceSensor) ungur).getDistance(DistanceUnit.MM));
                }
                telemetry.addData("AlBaStRu",ungur.red());
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
                telemetry.addData("slider poz1", slider1.getCurrentPosition());
                telemetry.addData("slider poz2", slider1.getCurrentPosition());
                telemetry.addData("slider power1", slider1.getPower());
                telemetry.addData("slider power2", slider2.getPower());
                telemetry.addData("brat poz:", arm1.getCurrentPosition());
                telemetry.update();
            }
        }
    });
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 235);
        crThreshHigh = inValues(crThreshHigh, 0, 235);
        cbThreshLow = inValues(cbThreshLow, 0, 235);
        cbThreshHigh = inValues(cbThreshHigh, 0, 235);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void RotireKindaSmooth(int poz, double power, int choice){
        if(choice%4==0) {
            motorFR.setTargetPosition(poz);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFR.setPower(power);
        }
        else if(choice%4==1) {
            motorFL.setTargetPosition(poz);

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFL.setPower(power);
        }
        else if(choice%4==2) {
            motorBR.setTargetPosition(poz);

            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBR.setPower(power);
        }
        else if(choice%4==3) {
            motorBL.setTargetPosition(poz);

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBL.setPower(power);
        }
    }
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);

        motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
        motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
        motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
        motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        while(motorFR.isBusy() && motorFL.isBusy() && motorBR.isBusy() && motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t < System.currentTimeMillis()){

        }
    }
    public void filament(int fraier){
        lastTime = System.currentTimeMillis();
        while ((lastTime + fraier > System.currentTimeMillis())){

        }
    }
    private ElapsedTime
            runtime = new ElapsedTime();

    public void Burdu(int poz, double pow, DcMotorEx motor,  double poz_servo){

        //sculantul.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
        motor.setTargetPosition(poz);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        balanganitor.setPosition(poz_servo);
        while(motor.isBusy()){}
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


    }
    public void Messi (int poz, double pow, DcMotorEx motor,  double poz_servo){

        //sculantul.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
        motor.setTargetPosition(poz);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while(motor.isBusy()){}
        balanganitor.setPosition(poz_servo);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


    }



    public void anaconda(int poz1,int poz2, double pow,int bani){
        slider1.setTargetPosition(poz1);
        slider2.setTargetPosition(poz2);
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider1.setPower(pow);
        slider2.setPower(pow);
        while (slider1.isBusy() && slider2.isBusy()){

        }
//
        lastTime=System.currentTimeMillis();
        while(lastTime + bani > System.currentTimeMillis()){

        }
        slider1.setPower(0);
        slider2.setPower(0);
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void schumy(int fata_spate, int stanga_dreapta, int emil,  int filament ){
        Translatare(0,fata_spate,0.3);
        kdf(200);
        Translatare(stanga_dreapta,0,0.8);
        kdf(200);
        anaconda(450,450,0.4,200);
//                while (ungur.red() < 400){
//                    Translatare(0,3,1);
//                }
        kdf(200);
//                while (ungur.red() < 500){
//                    Translatare(0,3,0.4);
//                }
        loader1.setPosition(0);
        kdf(200);
//        while (ungur.red() < 900){
//            Translatare(0,8,0.3);
//        }
        kdf(200);
        Translatare(0,-fata_spate - 25,0.3);
        kdf(200);
        Burdu(-1320,0.65, arm1,0.95);
        kdf(200);
        Rotire(emil,0.3);
        kdf(200);
        Translatare(0,filament,0.3);

    }







}
/*              |
                |
                |
                |
                |
________________|________________
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |

 */

    /*
    H/W(big=1):1.588
    H/W(small=3):0.23
    H/W(medium=2):4.23
     */
