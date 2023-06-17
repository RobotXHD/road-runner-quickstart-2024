//teleop robot nou
package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class TT_test_tampit extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorBR;
    public DcMotorEx motorBL;
    public DcMotorEx motorFL;
    public DcMotorEx motorFR;
    public DcMotorEx slider1;
    public DcMotorEx slider2;
    public Servo loader1;
    public Servo loader2;
    public Servo balanganitor;
    public DcMotorEx arm1;
    public DcMotorEx arm2;
    public boolean stop = false;
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
//                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
//                slider1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
//                slider2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
//                boolean ceva = false;
//                if(gamepad2.right_stick_y != 0.0){
//                    arm1.setPower(Config.armcoefficient * gamepad2.right_stick_y);
//                    arm2.setPower(Config.armcoefficient * gamepad2.right_stick_y);
//                    ceva = true;
//                }
//                else{
//                    if(ceva){
//                        ceva = false;
//                        pid.setSetpoint(arm1.getCurrentPosition());
//                    }
//                    pidResult = pid.performPID(arm1.getCurrentPosition());
//                    arm1.setPower(pidResult);
//                    arm2.setPower(pidResult);
//                }


                slider1.setPower(gamepad2.left_stick_y);
                slider2.setPower(gamepad2.left_stick_y);



                if (gamepad2.y) {
                    loader1.setPosition(1);
                    loader2.setPosition(0.6);
                }

                if (gamepad2.a) {
                    loader1.setPosition(0.5);
                    loader2.setPosition(0.9);
                }

                if (gamepad2.dpad_up) {
                    balanganitor.setPosition(0.5);
                }

                if (gamepad2.dpad_down) {
                    balanganitor.setPosition(0.9);
                }
            }
        }
    });
    public double corection,error,pidResult;
    public int targetpos;
    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    public boolean ok = true;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                if (abs(pmotorFL) > max) {
                    max = abs(pmotorFL);
                }
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
                //SLOW-MOTION
                //SLOW-MOTION
                if (gamepad1.left_trigger > 0) {
                    sm = 2;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_trigger > 0) {
                        sm = 5;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    } else {
                        sm = 0.5;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    }
                }
            }
        }
    });
    //zip_tie_ma_tai f = new zip_tie_ma_tai(true);
    private Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    private final Thread dip = new Thread(new Runnable() {
        @Override
        public void run() {
            boolean setSetpoint = true;
            pid.enable();
            while(!stop){
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
                arm1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
                arm2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
                if(gamepad2.right_stick_y != 0.0){
                    arm1.setPower(Config.armcoefficient * gamepad2.right_stick_y);
                    arm2.setPower(Config.armcoefficient * gamepad2.right_stick_y);
                    setSetpoint = true;
                }
                else{
                    if(setSetpoint){
                        setSetpoint = false;
                        pid.setSetpoint(arm1.getCurrentPosition());
                    }
                    pidResult = pid.performPID(arm1.getCurrentPosition());
                    arm1.setPower(pidResult);
                    arm2.setPower(pidResult);
                }
                if(gamepad2.x){
                    arm1.setTargetPosition(-162);
                    arm2.setTargetPosition(-162);
                    arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm1.setPower(0.8);
                    arm2.setPower(0.8);
                    while(arm1.isBusy() && arm2.isBusy()){}
                    arm1.setPower(0);
                    arm2.setPower(0);
                    arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pid.setSetpoint(arm1.getCurrentPosition());
                    pidResult = pid.performPID(arm1.getCurrentPosition());
                    arm1.setPower(pidResult);
                    arm2.setPower(pidResult);
                }
                if (gamepad2.b){
                    arm1.setTargetPosition(-23);
                    arm2.setTargetPosition(-23);
                    arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm1.setPower(0.8);
                    arm2.setPower(0.8);
                    while(arm1.isBusy() && arm2.isBusy()){}
                    arm1.setPower(0);
                    arm2.setPower(0);
                    arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pid.setSetpoint(arm1.getCurrentPosition());
                    pidResult = pid.performPID(arm1.getCurrentPosition());
                    arm1.setPower(pidResult);
                    arm2.setPower(pidResult);
                }
            }
        }
    });

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        slider1 = hardwareMap.get(DcMotorEx.class, "slider1");
        slider2 = hardwareMap.get(DcMotorEx.class, "slider2");
        loader1 = hardwareMap.servo.get("loader1");
        loader2 = hardwareMap.servo.get("loader2");
        balanganitor = hardwareMap.servo.get("balanganitor");
        arm1 = (DcMotorEx) hardwareMap.dcMotor.get("arm1");
        arm2 = (DcMotorEx) hardwareMap.dcMotor.get("arm2");

        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt folosite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        //motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);

        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        //Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        System.start();
        dip.start();
    }

    public void stop() {
        stop = true;
    }

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Brat pozitie1: ", arm1.getCurrentPosition());
        telemetry.addData("Brat pozitie2: ", arm2.getCurrentPosition());
        telemetry.addData("Pozitie slider1: ", slider1.getCurrentPosition());
        telemetry.addData("Pozitie slider2: ", slider2.getCurrentPosition());
        telemetry.addData("MOTORBL: ", motorBL.getCurrentPosition());
        telemetry.addData("MOTORBR: ", motorBR.getCurrentPosition());
        telemetry.addData("MOTORFL: ", motorFL.getCurrentPosition());
        telemetry.addData("MOTORFR: ", motorFR.getCurrentPosition());
        telemetry.addData("GP", gamepad2.right_stick_y);
        telemetry.addData("MotorSetpoint: ",Config.armcoefficient * gamepad2.right_stick_y);
        telemetry.addData("MotorVelocity: ", arm1.getVelocity());
        telemetry.addData("MotorPower", arm1.getPower());
        telemetry.addData("error:",pid.getError());
        telemetry.addData("getSetpoint:",pid.getSetpoint());
        telemetry.addData("Perror:",pid.getError() * pid.getP());
        telemetry.addData("Ierror:",pid.getISum() * pid.getI());
        telemetry.addData("Derror:",pid.getDError() * pid.getD());
        telemetry.addData("Corectie", pidResult);
        telemetry.update();
    }


    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

/*  __          _
 / _|        | |
| |_ ___  ___| |_
|  _/ _ \/ _ \ __|
| ||  __/  __/ |_
|_| \___|\___|\__|

 */