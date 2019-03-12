package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

class HardwareInitialiser {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor rotor;

    ElapsedTime runtime = new ElapsedTime();

    HardwareInitialiser(HardwareMap hwMap){
        this.leftMotor  = hwMap.get(DcMotor.class, "motor 0");
        this.rightMotor = hwMap.get(DcMotor.class, "motor 1");
        this.rotor      = hwMap.get(DcMotor.class, "motor 2");

        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rotor.setDirection(DcMotor.Direction.FORWARD);

        this.leftMotor.setPower(0);
        this.rightMotor.setPower(0);
        this.rotor.setPower(0);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

