package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "FarAuto")
public class FarAuto extends NextFTCOpMode {
    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    public int state = 1;
    public FarAuto() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left");
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();

    public void runAllMotors(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    public void zeroMotors() {
        runAllMotors(0,0,0,0);
    }

    @Override
    public void onStartButtonPressed() {
        timer1.reset();
        timer2.reset();
        runAllMotors(0.5,0.5,0.5,0.5);
    }

    @Override
    public void onUpdate() {
        switch (state) {
            case 1:
                if (timer1.milliseconds() > 750) {
                    runAllMotors(0,0,0,0);
                    state = 3;
                }
                break;
            case 3:

        }
    }
}
