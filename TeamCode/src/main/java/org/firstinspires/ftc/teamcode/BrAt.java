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
public class BrAt extends OpMode {
    public DcMotorEx arm1;
    public boolean stop = false;
    @Override
    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        
    }

    @Override
    public void loop() {
        telemetry.addData("pozitie bratt", arm1.getCurrentPosition());

    }

    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                arm1.setPower(gamepad2.right_stick_y);
            }
        }
    });

    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */

        Systems.start();
        

    }
}
