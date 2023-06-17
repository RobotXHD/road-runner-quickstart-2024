package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class notebook extends LinearOpMode {

    Servo servo_miscare;
    DcMotorEx arm;
    double pidResult;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(Var.kp,Var.kd,Var.ki);
    @Override
    public void runOpMode() throws InterruptedException {
        PID(1200,0.77,arm,0.5);
    }
    public void PID(int poz, double pow, DcMotorEx motor, double poz_servo){

        motor.setTargetPosition(poz);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        servo_miscare.setPosition(poz_servo);
        while(motor.isBusy()){}
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidResult = pid.performPID(motor.getCurrentPosition());
        motor.setPower(pidResult);

    }

    };

