package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class RemoteOpMode extends LinearOpMode {

    private DcMotor motorL;
    private DcMotor motorR;

    @Override
    public void runOpMode(){
        motorL = hardwareMap.get(DcMotor.class, "motor 0 HD hex");
        motorR = hardwareMap.get(DcMotor.class, "motor 2 HD hex");

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        motorL.setDirection(DcMotor.Direction.FORWARD);
        motorR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double powerL;
        double powerR;
        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            double direction =  gamepad1.right_stick_x;

            powerL = Range.clip(speed + direction, -1.0, 1.0) ;
            powerR = Range.clip(speed - direction, -1.0, 1.0) ;

            motorL.setPower(powerL);
            motorR.setPower(powerR);

            telemetry.addData("Left motor power", motorL.getPower());
            telemetry.addData("Right motor power", motorR.getPower());
            telemetry.update();
        }
    }
}
