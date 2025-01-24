package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "WgjTeleOp", group = "TeleOp")
public class WgjTeleOp extends OpMode {

    private DcMotor motorLeftFront, motorLeftBack, motorRightFront, motorRightBack;

    @Override
    public void init() {
        // 初始化电机
        motorLeftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        motorLeftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        motorRightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        motorRightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");

        // 设置电机方向
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // 获取左摇杆（前后移动）和右摇杆（左右旋转）的输入
        double drive = Math.pow(gamepad1.left_stick_y,2); // 前后移动
        double turn = Math.pow(-gamepad1.right_stick_x,2); // 左右旋转（取反修正方向）

        // 获取 ZL 和 ZR 按键（用于左右平移）
        double strafe = 0;
        double strafeSpeed = 0; // 初始化平移速度

        // 根据触发键的力度决定平移方向和速度
        if (gamepad1.left_trigger > 0.05) { // ZL 按钮
            strafe = -1;  // 向左平移
            strafeSpeed = Math.pow(gamepad1.left_trigger, 2);
        } else if (gamepad1.right_trigger > 0.05) { // ZR 按钮
            strafe = 1;   // 向右平移
            strafeSpeed = Math.pow(gamepad1.right_trigger, 2);
        }

        // 计算电机动力，加入平移速度
        double leftFrontPower = drive - strafe * strafeSpeed + turn;   // 左前电机
        double leftBackPower = drive + strafe * strafeSpeed + turn;    // 左后电机
        double rightFrontPower = drive + strafe * strafeSpeed - turn;  // 右前电机
        double rightBackPower = drive - strafe * strafeSpeed - turn;   // 右后电机

        // 急停逻辑：当所有输入接近 0 时，停止所有电机
        if (Math.abs(drive) < 0.05 && Math.abs(turn) < 0.05 && Math.abs(strafeSpeed) < 0.05) {
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            telemetry.addData("Status", "Emergency Stop - No Input");
        } else {
            // 设置电机的动力
            motorLeftFront.setPower(leftFrontPower);
            motorLeftBack.setPower(leftBackPower);
            motorRightFront.setPower(rightFrontPower);
            motorRightBack.setPower(rightBackPower);

            // 显示调试信息
            telemetry.addData("Left Front Power", leftFrontPower);
            telemetry.addData("Left Back Power", leftBackPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.addData("Right Back Power", rightBackPower);
            telemetry.addData("Strafe Speed", strafeSpeed); // 显示当前的平移速度
        }

        telemetry.update();
    }
}
