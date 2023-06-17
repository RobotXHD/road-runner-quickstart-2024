// power play raventech
// hai corvinu
// 11.09.2022(herban, maia)

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
/*@TeleOp face ca programul sa apara in configuratia driver hub-ului/telefonului cu aplicatia de driver station, in partea de TeleOp. */
@TeleOp
/*Linia asta de cod incepe cu numele programului(FoundationTeleOp) si la sfarsitul liniei este tipul de program:
    OpMode = TeleOp
    LinearOpMode = Autonom
  Linia de cod va da eroare dupa ce o scrii, doar apasa pe cod, apasa pe beculetul rosu si apoi apasa pe implement methods, asta va importa functiile de init si loop.
  Functia de init se declanseaza numai o data, dar cea de loop se repeta incontinuu si este locul unde se pun functiile care misca robotul in general, sau face telemetrie in cazul asta.
 */
public class PowerPlay extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorBR;
    public DcMotorEx motorBL;
    public DcMotorEx motorFL;
    public DcMotorEx motorFR;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    public DcMotorEx arm1;
    public DcMotorEx arm2;
    public DcMotorEx sfoara;
    Servo loader13;
    Servo loader1;
    Servo loader2;
    Servo costa;
    boolean stop = false;
    boolean apasat = false;
    public String trivago = "Trivago";
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;


    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        arm1     = (DcMotorEx) hardwareMap.dcMotor.get("arm1");
        arm2     = (DcMotorEx) hardwareMap.dcMotor.get("arm2");
        sfoara   = (DcMotorEx) hardwareMap.dcMotor.get("sfoara");

        loader2 =  hardwareMap.servo.get("balanganitor");
        loader1 = hardwareMap.get(Servo.class, "gheara1" );
        //loader13 = hardwareMap.get(Servo.class, "gheara2");
        loader13 = hardwareMap.servo.get("gheara2");
        costa = hardwareMap.servo.get("costa");



        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt folosite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);


        
        arm2.setDirection(DcMotorEx.Direction.REVERSE);

        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sfoara.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        /*motorFR.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotorEx.RunMode.RESET_ENCODERS);*/

        arm1.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        arm2.setMode(DcMotorEx.RunMode.RESET_ENCODERS);
        sfoara.setMode(DcMotorEx.RunMode.RESET_ENCODERS);



        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
        /*motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/

        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sfoara.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        /*Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        /*motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sfoara.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



    }
    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start(){
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        Systems.start();
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = -y - x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = -y - x - rx;
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
                if (gamepad1.left_bumper ) {
                    sm = 1.5;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_bumper) {
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

    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            costa.setPosition(0);
            while (!stop) {

                if (gamepad2.back)
                    arm1.setMode(DcMotor.RunMode.RESET_ENCODERS);
                if (gamepad2.back)
                    arm2.setMode(DcMotor.RunMode.RESET_ENCODERS);
                if (gamepad2.back)
                    sfoara.setMode(DcMotor.RunMode.RESET_ENCODERS);



                //cod pentru ridicarea bratului
                arm1.setPower(gamepad2.right_stick_y);
                arm2.setPower(gamepad2.right_stick_y);

                //extinderea sliderului
                sfoara.setPower(-gamepad2.left_stick_y);

                //gheara
                if (gamepad2.y) {
                    loader1.setPosition(1);
                    loader13.setPosition(0.6);

                }
                if (gamepad2.a) {
                    loader1.setPosition(0.5);
                    loader13.setPosition(0.9);

                }



                //balanganirea ghearei
                if (gamepad2.dpad_up) {
                    costa.setPosition(0);
                    loader2.setPosition(0.5);
                }

                if (gamepad2.dpad_down) {

                    loader2.setPosition(0.05);
                }

                if (gamepad2.dpad_left){
                    sfoara.setTargetPosition(2300);
                    apasat = true;
                    sfoara.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sfoara.setPower(0.7);
                    while(sfoara.isBusy()) ;
                    sfoara.setPower(0);
                    sfoara.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }









            }

        }

    });



    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry.addData("Hotel:", trivago);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Brat pozitie: ", arm1.getCurrentPosition());
        telemetry.addData("Pozitie slider: ", sfoara.getCurrentPosition());
        telemetry.addData("MOTORBL: ", motorBL.getCurrentPosition());
        telemetry.addData("MOTORBR: ", motorBR.getCurrentPosition());
        telemetry.addData("MOTORFL: ", motorFL.getCurrentPosition());
        telemetry.addData("MOTORFR: ", motorFR.getCurrentPosition());
        telemetry.update();

    }
    /*Functia asta face ca toate motoarele a ruleze cu o anumita putere;
    Functiile sunt linii de cod comprimate in una singura, ceea ce este foarte fain daca vrei sa faci o secventa de linii de cod de mai multe ori. De asemenea, cand apelezi o functie, trebuie sa scrii si parametrii ei, daca exista.*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
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