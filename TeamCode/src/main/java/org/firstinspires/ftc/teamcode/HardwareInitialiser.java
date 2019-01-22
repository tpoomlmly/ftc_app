package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

class HardwareInitialiser
{
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    DcMotor hook;
    DcMotorSimple grabber;

    ElapsedTime runtime  = new ElapsedTime();

    HardwareInitialiser(HardwareMap hwMap){
        this.leftMotor  = hwMap.get(DcMotor.class, "motor 0");
        this.rightMotor = hwMap.get(DcMotor.class, "motor 1");
        this.arm        = hwMap.get(DcMotor.class, "motor 2");
        //this.hook       = hwMap.get(DcMotor.class, "motor 3");
        this.grabber    = hwMap.get(DcMotorSimple.class, "servo 0");

        this.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.grabber.setDirection(DcMotorSimple.Direction.FORWARD);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //this.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftMotor.setPower(0);
        this.rightMotor.setPower(0);
        this.arm.setPower(0);
        //this.hook.setPower(0);
        this.grabber.setPower(0);
    }
 }

