
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.util.ColorMatchResult;
//import com.qualcomm.robotcore.util.ColorMatch;

@Autonomous
public class Bilantul_energiilor extends LinearOpMode {
    //de fapt AutonomousA2
    private OpenCvWebcam webcam;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private double height, width;
    private Rect rect;
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
    //ColorSensor ungur;

//    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
//            = RevHubOrientationOnRobot.LogoFacingDirection.values();
//    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
//            = RevHubOrientationOnRobot.UsbFacingDirection.values();
//    static int LAST_DIRECTION = logoFacingDirections.length - 1;
//    static float TRIGGER_THRESHOLD = 0.2f;
//
//    public static double error = 5.1;
//    public static double p = .02;
//    public static double botHeading;
//    public static double reference = botHeading;


//    int logoFacingDirectionPosition;
//    int usbFacingDirectionPosition;
//    boolean orientationIsValid = true;
//    YawPitchRollAngles orientation;

    double power = 1;
    int position = 0;



    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    double last_pos_servo;
    boolean merge = false;
    boolean merge2 = false;
    FunctiiDeAutonom f = new FunctiiDeAutonom(true);
    MechShow a = new MechShow(true);
    private BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    private double  leftSpeed2     = 0;
    private double  rightSpeed2    = 0;
    private int     leftTarget2    = 0;
    private int     rightTarget2   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);

//    static final double COUNTSPERR = 383.6;
//    static final double GEARREDUCTION = 1;
//    static final double DIAMROT = 9.6;
//    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    public boolean next = false;
    public boolean bagPula = false;

    public double start_time;
    public double  stop_time;
    @Override
    public void runOpMode() throws InterruptedException {
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
        //        ungur = hardwareMap.get(ColorSensor.class, "senzor_gheara");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;






        //arm2.setDirection(DcMotorEx.Direction.REVERSE);
        //motorBR.setDirection(DcMotorEx.Direction.REVERSE);
        //motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        slider2.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);


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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        CV_detectionType = DetectionTypes.DAY_green;

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
//        while (opModeInInit()) {
//            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//            telemetry.update();
//        }

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
                rect = pipeline.getRect();
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
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
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
//    void updateOrientation() {
//        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
//        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
//        try {
//            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
//            imu.initialize(new IMU.Parameters(orientationOnRobot));
//            orientationIsValid = true;
//        } catch (IllegalArgumentException e) {
//            orientationIsValid = false;
//        }
//    }


    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            kdf(200);
//            loader2.setPosition(0.9);
//            loader1.setPosition(0.5);
            //balanganitor.setPosition(0.2);
            loader1.setPosition(0.05);



            if(varrez=="Dreapta" && !isStopRequested()) {
                //imu.resetYaw();
                //108% - 90+

                //parcare
                CV_detectionType = DetectionTypes.DAY_yellow;
                next = false;
                Translatare(0, -165, 0.55);
                turnToHeading(TURN_SPEED, 30.0);
                holdHeading(TURN_SPEED, 30.0,0.5);

                //                CV_detectionType = DetectionTypes.DAY_yellow;
                start_time = System.currentTimeMillis();
                while (pipeline.getRect() == null){}
                stop_time = System.currentTimeMillis();
                telemetry.addData("Time", stop_time - start_time );
                telemetry.update();
                filament(1000);
//                while (error > 50){
//                    error = Webcam_w/2.0 - rect.x;
//                    moveRobot(0,TURN_SPEED/2);
//                }
                //pipeline.getRectX() > 250 && pipeline.getRectX() < 450
                bagPula = true;

                double MAX_CENTER_DISTANCE = 50;
                double objectCenter_x;
                double objectCenter_y;
                double imageCenter_x;
                double imageCenter_y;

                objectCenter_x = pipeline.getRect().x + pipeline.getRect().width / 2.0;
                objectCenter_y =  pipeline.getRect().y + pipeline.getRect().height / 2.0;

                imageCenter_x = Webcam_w / 2.0;
                imageCenter_y = Webcam_h / 2.0;

                while (!(imageCenter_x - objectCenter_x < MAX_CENTER_DISTANCE)){
                    objectCenter_x = pipeline.getRect().x + pipeline.getRect().width / 2.0;
                    moveRobot(0,TURN_SPEED/3);

                }


//                if (imageCenter_x - objectCenter_x  < MAX_CENTER_DISTANCE && imageCenter_y - objectCenter_y < MAX_CENTER_DISTANCE) {
//                    // The object is centered
//                } else {
//                    // The object is not centered
//                }
//                moveRobot(0,0);
//                turnToHeading( TURN_SPEED, 90.0);               // Turn  CW to -45 Degrees
//                holdHeading( TURN_SPEED, 90.0, 0.5);






//                Burdu(-1360,0.65,arm1,0.2);
//                kdf(200);
//                anaconda(1400,1400,0.8,400);
//                anaconda(1,1,0.6,10);
//                while (true){
//                    telemetry.addData("slider poz1", slider1.getCurrentPosition());
//                    telemetry.addData("slider poz2", slider1.getCurrentPosition());
//                }



//                //cod bun
//                Translatare(0,-165,0.3);
//                filament(200);
//                Rotire(-62,0.2);
//                //Rotire(100,0.4);
////                kdf(200);
////                //Messi(-1360,0.7, arm1,0.95);
////                //balanganitor.setPosition(0.95);
////                kdf(200);
//                filament(150);
//                Translatare(0,-30,0.2);
//                filament(300);
//                anaconda(1400,1400,0.7,1400);
//                kdf(200);
//                anaconda(1,1,0.4,0);
////                kdf(200);
//                loader1.setPosition(0.3);
//                filament(300);
//                Burdu(-20,0.77,arm1,0.92);
//                filament(200);
//                Rotire(-107,0.3);
//                filament(200);
//                Translatare(-5,0,0.3);
//                filament(300);
//                Translatare(0,-70,0.2);



            }

            //else
            else if(varrez == "Mijloc" && !isStopRequested()) {

                //imu.resetYaw();
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
                Translatare(0,-165,0.3);
                filament(200);
                Rotire(-62,0.2);
                //Rotire(100,0.);
//                kdf(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                //balanganitor.setPosition(0.95);
//                kdf(200);
                filament(150);
                Translatare(0,-30,0.2);
                filament(300);
                anaconda(1400,1400,0.4,1400);
                kdf(200);
                anaconda(1,1,0.7,0);
//                kdf(200);
                loader1.setPosition(0.3);
                filament(300);
                Burdu(-20,0.77,arm1,0.92);
                filament(200);
                Rotire(-107,0.3);
                filament(200);
                Translatare(-5,0,0.3);
                filament(300);
                Translatare(0,9,0.2);



            }

            else if(varrez == "Stanga"  && !isStopRequested()) {
                //imu.resetYaw();
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
                Translatare(0,-165,0.3);
                filament(200);
                Rotire(-62,0.2);
                //Rotire(100,0.4);
//                kdf(200);
//                //Messi(-1360,0.7, arm1,0.95);
//                //balanganitor.setPosition(0.95);
//                kdf(200);
                filament(150);
                Translatare(0,-30,0.2);
                filament(300);
                anaconda(1400,1400,0.7,1400);
                kdf(200);
                anaconda(1,1,0.4,0);
//                kdf(200);
                loader1.setPosition(0.3);
                filament(300);
                Burdu(-20,0.77,arm1,0.92);
                filament(200);
                Rotire(-107,0.3);
                filament(200);
                Translatare(-5,0,0.3);
                filament(300);
                Translatare(0,93,0.2);

            }

        }
    });

    public Thread mafutinel = new Thread(new Runnable() {
        @Override
        public void run() {
            filament(90);
            //Messi(-1430,0.6, arm1,0.2);


        }
    });

    public  Thread muie = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()){
                if (merge){
                    filament(260);
                    Messi(-1445,0.7, arm1,0.05);
                    merge = false;
                }


                //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                //AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//                if (ungur instanceof DistanceSensor){
//                    telemetry.addData("DiStAnTa", ((DistanceSensor) ungur).getDistance(DistanceUnit.MM));
//                }
//                telemetry.addData("AlBaStRu",ungur.blue());
//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
                telemetry.addData("slider poz1", slider1.getCurrentPosition());
                telemetry.addData("slider poz2", slider1.getCurrentPosition());
                telemetry.addData("slider power1", slider1.getPower());
                telemetry.addData("slider power2", slider2.getPower());
                telemetry.addData("brat poz:", arm1.getCurrentPosition());
                telemetry.addData("rect x", rect.x);
                telemetry.addData("rect y", rect.y);
                telemetry.addData("WebcamW", Webcam_w);
                telemetry.addData("WebcamH", Webcam_h);

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
        next = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;
        

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((deltaY - deltaX) * cpcm);

        motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
        motorBL.setTargetPosition(currentmotorBL + (int) ((deltaY + deltaX) * cpcm));
        motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
        motorFL.setTargetPosition(currentmotorFL + (int) ((deltaY - deltaX) * cpcm));

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motorBL.setPower(speed);
//        motorBR.setPower(speed);
//        motorFL.setPower(speed);
//        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            if (motorBR.getCurrentPosition() < targetBR/1.4 ){
                speed = 0.3;
            }
            motorBL.setPower(speed);
            motorBR.setPower(speed);
            motorFL.setPower(speed);
            motorFR.setPower(speed);
            // obtain the encoder position

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

        next = true;
    }
    /*public void turn(int value) {
        power = 1;
        while (opModeIsActive() && botHeading < value) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            // obtain the encoder position
            botHeading = -orientation.getYaw(AngleUnit.DEGREES);
            // calculate the error
            error = reference - botHeading;

            if(botHeading > value/1.3) {
                power = .4;
            }
            //turn
            motorFR.setPower(power * .5);
            motorBR.setPower(power * .5);
            motorFL.setPower(power * .5);
            motorBL.setPower(power * .5);

            telemetry.addData("pose", botHeading);
            telemetry.addData("error", error);
            telemetry.addData("logo Direction (set with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
            telemetry.addData("usb Direction (set with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");
            telemetry.addData("lf", motorFL.getPower());
            telemetry.addData("rf", motorFR.getPower());
            telemetry.addData("target", reference);
            telemetry.addData("\n position", position);
            telemetry.update();
        }
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
    }*/
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
//                while (ungur.blue() < 400){
//                    Translatare(0,3,1);
//                }
        kdf(200);
//                while (ungur.blue() < 500){
//                    Translatare(0,3,0.4);
//                }
        loader1.setPosition(0);
        kdf(200);
//        while (ungur.blue() < 900){
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
    public void turnToHeading(double maxTurnSpeed, double heading) {
            // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            }

            // Stop all motion;

            moveRobot(0, 0);
            next = false;



    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime && next)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            }

            // Stop all motion;
            moveRobot(0, 0);
            next = false;


    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

            // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        motorFL.setPower(leftSpeed);
        motorBL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);
        motorBR.setPower(rightSpeed);

        next = false;
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      motorFL.getCurrentPosition(),
                    motorFR.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
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
/*
0 0 1
0 1 0
1 0 0
 */