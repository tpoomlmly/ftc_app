package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class RemoteOpMode extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    @Override
    public void runOpMode(){
        motorFL = hardwareMap.get(DcMotor.class, "motor 1 HD hex");
        motorFR = hardwareMap.get(DcMotor.class, "motor 3 HD hex");
        motorBL = hardwareMap.get(DcMotor.class, "motor 2 HD hex");
        motorBR = hardwareMap.get(DcMotor.class, "motor 0 HD hex");

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double powerL;
        double powerR;
        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            double direction = 2*gamepad1.left_trigger - 1; //On the REV gamepad the right stick x is mapped as the left trigger

            powerL = Range.clip(speed + direction, -1.0, 1.0) ;
            powerR = Range.clip(speed - direction, -1.0, 1.0) ;

            motorBL.setPower(powerL);
            motorFL.setPower(powerL);
            motorFR.setPower(powerR);
            motorBR.setPower(powerR);

            telemetry.addData("Left motor power", motorFL.getPower());
            telemetry.addData("Right motor power", motorFR.getPower());
            telemetry.update();
        }
    }
}