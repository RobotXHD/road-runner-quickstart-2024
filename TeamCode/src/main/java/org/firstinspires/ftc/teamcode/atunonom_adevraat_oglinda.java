package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.logging.Logger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous
@Disabled
public class atunonom_adevraat_oglinda extends LinearOpMode {
    /* Declare OpMode members. */
    /*
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    */
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotorEx DJL;
    private DcMotorEx arm;
    private DcMotorEx arm2;
    //private DcMotorEx //grip;
    //private DcMotorEx //shuter;
    private Servo loader1;
    private Servo loader2;
    private Servo grabber_left;
    private Servo grabber_right;
    //private Servo //stopper_left;
    //private Servo //stopper_right;
    private DistanceSensor Distanta;

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;
    boolean a=true;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);


    double Lpos = 0.7;

    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    String varrez;
    boolean rata = false;
    double lastTime;

    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR * GEARREDUCTION) / (DIAMROT * 3.1415);

    String rezultat = "None";
    String rezultat1 = "None";


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0, 0, 0.1, 0.1);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        motorBL = hardwareMap.get(DcMotor.class, "1"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "4"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "2"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "3"); // Motor Front-Right
        arm = (DcMotorEx) hardwareMap.dcMotor.get("gheara");
        arm2 = (DcMotorEx) hardwareMap.dcMotor.get("gheara2");
        DJL = (DcMotorEx) hardwareMap.dcMotor.get("roata");
        //grabber_left   = hardwareMap.servo.get("grabber_left");
        //grabber_right  = hardwareMap.servo.get("grabber_right");
        loader1 = hardwareMap.servo.get("cutie");
        loader2 = hardwareMap.servo.get("gheara 2");
        Distanta = hardwareMap.get(DistanceSensor.class, "distanta dreapta");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ////grip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ////shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DJL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //loader.setPosition(1.0);//0.32

//        //shuter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(constants.p, constants.i, constants.d, constants.f));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        if (pipeline.error) {
            telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
        }
        // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
        // testing(pipeline);

        // Watch our YouTube Tutorial for the better explanation
        while(!isStarted()) {
            try {
                double rectangleArea = pipeline.getRectArea();

                //Print out the area of the rectangle that is found.
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.addData("Position", "X = " + pipeline.getRectX() + "    Y = " + pipeline.getRectY());
                //Check to see if the rectangle has a large enough area to be a marker.
                if (rectangleArea > minRectangleArea) {
                    //Then check the location of the rectangle to see which barcode it is in.
                    if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Right");
                    } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Left");
                    } else {
                        telemetry.addData("Barcode Position", "Center");
                    }
                }
                if (pipeline.getRectX() < 150) {
                    varrez = "Stanga";
                } else if (pipeline.getRectX() > 150 && pipeline.getRectX() < 350) {
                    varrez = "Mijloc";
                } else if (pipeline.getRectX() > 350 && pipeline.getRectX() < 500) {
                    varrez = "Dreapta";
                }
                telemetry.addData("var", varrez);
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("E:", e.getMessage());
                telemetry.update();
            }
        }
        if(a==true) {
            a=false;
            if(!isStopRequested()&&isStarted()) {
                Atunonom.start();
            }
        }
        while(!isStopRequested()){

        }
    }
    public Thread Atunonom = new Thread(new Runnable(){
        @Override
        public void run() {
            if(varrez=="Dreapta") {
                loader1.setPosition(0);
                loader2.setPosition(0.17);

                arm.setTargetPosition(85);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while (arm.isBusy()) ;
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                Translatare(0, 88,0.4);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                loader1.setPosition(0.3);
                loader2.setPosition(0);
                  Translatare(0,-30 ,0.4 );
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    while (arm.isBusy()) ;
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lastTime = System.currentTimeMillis();
                    while (lastTime + 100 > System.currentTimeMillis()) {
                    }

                    Translatare(95,-108,0.4);
                DJL.setPower(-0.35);
                lastTime = System.currentTimeMillis();
                while (lastTime + 2900 > System.currentTimeMillis()) {
                }
                DJL.setPower(0);
                    Translatare(0,60,0.3);
                    Translatare(20,0,0.2);








            }

            if(varrez == "Mijloc")
            {
                loader1.setPosition(0);
                loader2.setPosition(0.17);

                arm.setTargetPosition(135);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while (arm.isBusy()) ;
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                Translatare(0, 88,0.4);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                loader1.setPosition(0.3);
                loader2.setPosition(0);
                Translatare(0,-30 ,0.4 );
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while (arm.isBusy()) ;
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }

                Translatare(95,-108,0.4);
                Translatare(0,-5,0.3,500);
                DJL.setPower(-0.35);
                lastTime = System.currentTimeMillis();
                while (lastTime + 2900 > System.currentTimeMillis()) {
                }
                DJL.setPower(0);
                Translatare(0,60,0.3);
                Translatare(20,0,0.2);

            }

            if(varrez == "Stanga") {
                loader1.setPosition(0);
                loader2.setPosition(0.17);

                arm.setTargetPosition(225 );
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while (arm.isBusy()) ;
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                Translatare(0, 88,0.4);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }
                loader1.setPosition(0.3);
                loader2.setPosition(0);
                Translatare(0,-30 ,0.4 );
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while (arm.isBusy()) ;
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while (lastTime + 100 > System.currentTimeMillis()) {
                }

                Translatare(95,-108,0.4);
                DJL.setPower(-0.35);
                lastTime = System.currentTimeMillis();
                while (lastTime + 2900 > System.currentTimeMillis()) {
                }
                DJL.setPower(0);
                Translatare(0,60,0.3);
                Translatare(20,0,0.2);


            }
        }
    });

    public void Translatare(int deltaX, int deltaY, double speed, int timeout) {
        boolean Done = false;
        int errorpos;
        int Maxerror = 20;
        double lastTime1;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) ((deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) ((deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
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
        lastTime1  = System.currentTimeMillis();
        Done = false;
        while (!Done && opModeIsActive() && lastTime1 + timeout > System.currentTimeMillis()) {
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

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Translatare(int deltaX, int deltaY, double speed) {
        boolean Done = false;
        int errorpos;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) ((deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) ((deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
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
        while (!Done && opModeIsActive()) {
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

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Rotire(int deltaA, double speed) {
        boolean Done = false;
        int errorpos;
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
        while (!Done && opModeIsActive()) {
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
}



