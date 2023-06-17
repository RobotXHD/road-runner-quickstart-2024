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
public class Muri_robot extends OpMode {
    public DcMotorEx motorR;
    public DcMotorEx motorL;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorL;
    double pmotorR;
    public boolean stop = false;

    @Override
    public void init() {
        motorL = hardwareMap.get(DcMotorEx.class, "motorL"); // Motor Back-Left
        motorR = hardwareMap.get(DcMotorEx.class, "motorR"); // Motor Back-Left

        motorL.setDirection(DcMotorEx.Direction.REVERSE);

        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        motorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {


    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {

            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = gamepad1.left_stick_y;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, c
                u ajutorul puterilor de la controller*/
                pmotorL = y  + rx;
                pmotorR = y -  rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                if (abs(pmotorL) > max) {
                    max = abs(pmotorL);
                }
                if (abs(pmotorR) > max) {
                    max = abs(pmotorR);
                }

                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorL /= max;
                    pmotorR /= max;

                }
                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
                //SLOW-MOTION
                //SLOW-MOTION
                if (gamepad1.left_bumper ) {
                    sm = 2;
                    POWER(pmotorR / sm, pmotorL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_bumper) {
                        sm = 5;
                        POWER(pmotorR / sm, pmotorL / sm );
                    } else {
                        sm = 0.5;
                        POWER(pmotorR / sm, pmotorL / sm );
                    }
                }

                // if (gamepad1.a){
                //  Translatare(45,0.6);
                // }
            }


        }


    });

    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();

    }


    public void stop() {
        stop = true;
    }


    public void POWER(double df1, double sf1) {
        motorR.setPower(df1);
        motorL.setPower(sf1);
    }
}
