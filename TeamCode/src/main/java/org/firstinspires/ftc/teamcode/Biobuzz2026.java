package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Biobuzz2026 extends LinearOpMode {

    // Sentido de giro (olhando de frente para o eixo do motor):
    // potência POSITIVA = anti-horário | potência NEGATIVA = horário
    static final double SHOOTER_HORARIO      = -0.7; // RB - 70%
    static final double SHOOTER_ANTIHORARIO  =  1.0; // LB - 100%
    static final double INTAKE_ANTIHORARIO   =  1.0; // D-pad cima - 100%
    static final double INTAKE_HORARIO       = -1.0; // D-pad baixo - 100%

    // Servo (portão)
    // Capacidade máxima do servo: REV Smart Robot Servo = 270 graus de fábrica
    static final double SERVO_CURSO_TOTAL = 270.0;
    // Limite de segurança: o servo NUNCA passa de 1/4 de volta
    static final double SERVO_LIMITE      = 90.0;
    // Quanto o portão abre ao apertar A (no máximo SERVO_LIMITE)
    static final double SERVO_ANGULO      = 88.0;
    static final double SERVO_POS_ZERO    = 0.0;
    // Com o limite aplicado, a escala 0..1 passa a valer 0..90 graus
    static final double SERVO_POS_88      = SERVO_ANGULO / SERVO_LIMITE; // ~0.978

    @Override
    public void runOpMode() throws InterruptedException {
        // ============ MECANUM - CONTROL HUB (gamepad1) ============
        // As strings entre aspas precisam bater com a configuração do Driver Hub.
        DcMotor frontLeft  = hardwareMap.dcMotor.get("Mecanum 1"); // porta 0 - frente esquerda
        DcMotor frontRight = hardwareMap.dcMotor.get("Mecanum 2"); // porta 1 - frente direita
        DcMotor backLeft   = hardwareMap.dcMotor.get("Mecanum 3"); // porta 2 - trás esquerda
        DcMotor backRight  = hardwareMap.dcMotor.get("Mecanum 4"); // porta 3 - trás direita

        // Inverte o lado direito.
        // Se o robô andar para trás quando mandar ir para frente,
        // inverta o lado esquerdo em vez do direito.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ============ MECANISMOS - EXPANSION HUB (gamepad2) ============
        DcMotor intake  = hardwareMap.dcMotor.get("Intake");
        DcMotor shooter = hardwareMap.dcMotor.get("Shooter");

        // Mantidos em FORWARD para o sentido seguir as constantes do topo
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake trava ao soltar; shooter desacelera sozinho
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Servo na porta 0 (nome precisa bater com a configuração do Driver Hub)
        Servo servo = hardwareMap.servo.get("Servo");
        // Se ao apertar A ele girar anti-horário, troque para REVERSE
        servo.setDirection(Servo.Direction.FORWARD);
        // Trava o servo entre 0 e 90 graus: qualquer posição pedida fica dentro desse intervalo
        servo.scaleRange(0.0, SERVO_LIMITE / SERVO_CURSO_TOTAL);

        // ============ IMU ============
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Ajuste conforme a montagem do hub no robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        // Ao apertar PLAY, o servo vai para o marco 0
        servo.setPosition(SERVO_POS_ZERO);

        while (opModeIsActive()) {
            // ============ MOVIMENTAÇÃO - GAMEPAD 1 ============
            double y = -gamepad1.left_stick_y; // eixo Y do analógico vem invertido
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Botão options (start no controle Xbox) zera a direção "frente do campo"
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotaciona o comando do joystick contra o ângulo do robô (field-centric)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // compensa o strafe imperfeito das mecanum

            // Mantém todas as potências entre -1 e 1 sem perder a proporção
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // ============ SHOOTER - GAMEPAD 2 ============
            // Segurando RB: horário 70% | Segurando LB: anti-horário 100% | Solto: parado
            double shooterPower = 0;
            if (gamepad2.right_bumper) {
                shooterPower = SHOOTER_HORARIO;
            } else if (gamepad2.left_bumper) {
                shooterPower = SHOOTER_ANTIHORARIO;
            }
            shooter.setPower(shooterPower);

            // ============ INTAKE - GAMEPAD 2 ============
            // D-pad cima: anti-horário 100% | D-pad baixo: horário 100% | Solto: parado
            double intakePower = 0;
            if (gamepad2.dpad_up) {
                intakePower = INTAKE_ANTIHORARIO;
            } else if (gamepad2.dpad_down) {
                intakePower = INTAKE_HORARIO;
            }
            intake.setPower(intakePower);

            // ============ SERVO - GAMEPAD 2 ============
            // A: gira 88 graus a partir do 0 | B: volta ao marco 0
            if (gamepad2.a) {
                servo.setPosition(SERVO_POS_88);
            } else if (gamepad2.b) {
                servo.setPosition(SERVO_POS_ZERO);
            }

            // ============ TELEMETRIA ============
            telemetry.addData("Heading (graus)", Math.toDegrees(botHeading));
            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("Shooter", "%.2f", shooterPower);
            telemetry.addData("Intake", "%.2f", intakePower);
            telemetry.addData("Servo (posição)", "%.3f", servo.getPosition());
            telemetry.update();
        }
    }
}