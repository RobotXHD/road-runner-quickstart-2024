package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@TeleOp
public class HAPCIU extends OpMode {
    public DcMotorEx motorBR;
    public DcMotorEx motorBL;
    public DcMotorEx motorFL;
    public DcMotorEx motorFR;
    public DcMotorEx brat;
    public DcMotorEx shimishmiey;
    public DcMotorEx aparatoare;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    public boolean stop = false;
    boolean mancarime = false;
    public boolean ceva = false;
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public double corection,error,pidResult;
    public int tagetpos;
    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    public boolean ok = true;

    @Override
    public void init() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        shimishmiey = hardwareMap.get(DcMotorEx.class, "invarte");
        aparatoare = hardwareMap.get(DcMotorEx.class, "poarta");

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shimishmiey.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aparatoare.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shimishmiey.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        aparatoare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shimishmiey.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        aparatoare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.addData("pozitie brat:", brat.getCurrentPosition());


    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {

            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = gamepad1.left_stick_y;
                x = -gamepad1.left_stick_x;
                rx = -gamepad1.right_stick_x;

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
                if (gamepad1.right_trigger > 0 ) {
                    sm = 1;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                   sm = 1.25;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }

                // if (gamepad1.a){
                //  Translatare(45,0.6);
                // }
            }


        }


    });
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            boolean plm = false;
            pid.enable();
            while (!stop){
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
                if(gamepad2.right_stick_y != 0.0){
                    brat.setPower(gamepad1.right_stick_y);
                    ceva = true;
                }
                else{
                    if(ceva){
                        ceva = false;
                        pid.setSetpoint(brat.getCurrentPosition());
                    }
                    pidResult = pid.performPID(brat.getCurrentPosition());
                    brat.setPower(pidResult);

                }if (gamepad1.right_bumper){
                    aparatoare.setPower(1);
                }
                else if (gamepad1.left_bumper){
                    aparatoare.setPower(-1);
                }
                if (gamepad1.right_stick_button){
                    shimishmiey.setPower(1);
                    Automatizare(1100,1,brat);
                }
                if (gamepad1.b){
                    shimishmiey.setPower(0);
                }
                if (gamepad1.left_stick_button){
                    Automatizare(-40,1,brat);
                    shimishmiey.setPower(0);
                }

            }

        }

        public void Automatizare(int poz, double pow, DcMotorEx motor){
            motor.setTargetPosition(poz);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while(motor.isBusy()){}
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }
            ceva = true;

        }
        public void Burdu_fara_burdu(int poz, double pow, DcMotorEx motor){
            motor.setTargetPosition(poz);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while(motor.isBusy()){}
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }
            ceva = true;

        }



    });

    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        System.start();

    }


    public void stop() {
        stop = true;
    }


    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}
