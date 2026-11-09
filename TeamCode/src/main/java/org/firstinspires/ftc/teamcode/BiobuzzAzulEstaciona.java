package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * AUTÔNOMO ALIANÇA AZUL — início em D6
 *
 * Arena (polegadas, origem no canto da plateia/lado vermelho, tiles de 24 pol):
 *   Início:  tile D6  -> centro do robô em (84, 132)
 *            FRENTE do robô (coletor) virada para a parede do FUNDO (y = 144)
 *            COSTAS do robô (shooter) viradas para a HIVE
 *
 *   Com o robô olhando para o fundo (+y):
 *     ir para a plateia (-y)          = robô anda de RÉ
 *     ir para a direita da arena (+x) = robô anda de lado para a DIREITA dele
 *
 *   Passo 1: lança as 3 bolas parado (60 pol até o centro da HIVE, igual a D1)
 *   Passo 2: lado para a DIREITA 42 pol -> x = 126 (coluna F, fora da HIVE)
 *   Passo 3: RÉ 96 pol (4 tiles)        -> y = 36 (linha 2)
 *   Passo 4: lado para a DIREITA 8 pol  -> x = 134, dentro da loading zone F2
 *            (F2: x 133–144, y 24–47)
 *   Fim:     estacionado, tudo antes de 30 s
 *
 * Folgas (robô de 18 pol):
 *   - Não dá para descer direto pela coluna D: a HIVE ocupa x 47–97, y 52,5–91,5.
 *   - Descendo em x = 126 o robô ocupa x 117–135: longe da HIVE (97) e com
 *     ~5 pol de folga da FLOWER da parede azul em (144, 96).
 *   - No fim, em (134, 36), ocupa x 125–143 e y 27–45: dentro da loading zone.
 */
@Autonomous(name = "Biobuzz-azul-estaciona", preselectTeleOp = "Biobuzz2026")
public class BiobuzzAzulEstaciona extends LinearOpMode {

    // ============ MESMOS AJUSTES DO TELEOP ============
    static final boolean INVERTER_STRAFE = true;

    // ============ TEMPO ============
    // O autônomo da FTC dura 30 s. Se chegar neste limite, o robô para onde estiver.
    static final double TEMPO_LIMITE = 29.0;

    // ============ SHOOTER E PORTÃO ============
    static final double SHOOTER_AUTONOMO = -0.7;
    static final double TEMPO_ACELERAR   = 5.0;   // espera do shooter ligar até a 1ª bola
    static final double TEMPO_DESLIGAR   = 1.2;   // shooter ainda gira isso depois da última bola, para ela sair
    static final int    QUANTIDADE_BOLAS = 3;
    static final double INTERVALO_BOLAS  = 3.5;
    // Se NENHUMA bola cair, aumente (ex.: 0.25). Se caírem DUAS, diminua (ex.: 0.10).
    static final double TEMPO_PORTAO_ABERTO = 0.20;

    static final double SERVO_CURSO_TOTAL = 270.0;
    static final double SERVO_LIMITE      = 90.0;
    static final double SERVO_ANGULO      = 50.0;
    static final double PORTAO_ABERTO     = 0.0;
    static final double PORTAO_FECHADO    = SERVO_ANGULO / SERVO_LIMITE;

    // ============ PERCURSO (em polegadas) ============
    // D6 (x=84) -> x=126: sai da frente da HIVE sem encostar na FLOWER da parede.
    static final double DISTANCIA_LADO_1 = 42.0;
    // y=132 -> y=36: 4 tiles descendo pela coluna F.
    static final double DISTANCIA_RE     = 96.0;
    // x=126 -> x=134: entra na loading zone F2.
    // Se bater na parede, diminua. Se não entrar na loading zone, aumente.
    static final double DISTANCIA_LADO_2 = 8.0;

    static final double POTENCIA_FRENTE = 0.5;
    static final double POTENCIA_LADO   = 0.5;

    // ============ CALIBRAÇÃO (IMPORTANTE!) ============
    // Quantas polegadas o robô anda em 1 segundo nas potências acima.
    // Como medir: coloque CALIBRAR = true, rode o autônomo, meça com trena
    // quanto o robô andou em 1 s para frente e em 1 s de lado, e coloque aqui.
    static final boolean CALIBRAR = false;
    static final double POL_POR_SEG_FRENTE = 20.0; // valor de exemplo, MEÇA o seu
    static final double POL_POR_SEG_LADO   = 16.0; // de lado o mecanum costuma ser mais lento

    // Pausa entre um movimento e outro, para o robô assentar
    static final double PAUSA_ENTRE_MOVIMENTOS = 0.3;

    // ============ CONTROLE ============
    static final double KP_RETO = 0.03;
    static final double KP_GIRO = 0.02;
    static final double GIRO_POTENCIA_MIN = 0.15;
    static final double GIRO_POTENCIA_MAX = 0.6;
    static final double GIRO_TOLERANCIA   = 2.0;

    DcMotor frontLeft, frontRight, backLeft, backRight, shooter, intake;
    Servo servo;
    IMU imu;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime tempoTotal = new ElapsedTime();
    int bolasSoltas = 0;
    String etapaAtual = "INIT";

    double alvoHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ============ HARDWARE (mesmos nomes do TeleOp) ============
        frontLeft  = hardwareMap.dcMotor.get("Mecanum 1");
        frontRight = hardwareMap.dcMotor.get("Mecanum 2");
        backLeft   = hardwareMap.dcMotor.get("Mecanum 3");
        backRight  = hardwareMap.dcMotor.get("Mecanum 4");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.dcMotor.get("Shooter");
        intake  = hardwareMap.dcMotor.get("Intake");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get("Servo");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.scaleRange(0.0, SERVO_LIMITE / SERVO_CURSO_TOTAL);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        servo.setPosition(PORTAO_FECHADO);

        telemetry.addLine(CALIBRAR ? "MODO CALIBRACAO" : "AZUL: robo em D6, coletor para o fundo, shooter para a HIVE.");
        telemetry.addLine("Aperte PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();
        alvoHeading = 0;
        tempoTotal.reset();

        if (CALIBRAR) {
            calibrar();
            pararTudo();
            return;
        }

        // =====================================================
        // ============ SEQUÊNCIA DO AUTÔNOMO AZUL =============
        // =====================================================

        // ---- 1) LANÇAR AS BOLAS (parado em D6) ----
        etapaAtual = "Acelerando shooter";
        servo.setPosition(PORTAO_FECHADO);
        shooter.setPower(SHOOTER_AUTONOMO);
        esperar(TEMPO_ACELERAR);

        for (int i = 1; i <= QUANTIDADE_BOLAS && opModeIsActive(); i++) {
            if (i > 1) {
                etapaAtual = "Intervalo entre bolas";
                esperar(INTERVALO_BOLAS);
            }
            etapaAtual = "Soltando bola " + i;
            soltarUmaBola();
            bolasSoltas = i;
        }

        etapaAtual = "Ultima bola saindo";
        esperar(TEMPO_DESLIGAR);
        shooter.setPower(0);

        // ---- 2) D6 -> coluna F: lado para a DIREITA do robô (= direita da arena) ----
        etapaAtual = "Lado ate coluna F";
        andarPolegadas(0, POTENCIA_LADO, DISTANCIA_LADO_1);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 3) Desce a coluna F: RÉ (shooter vai na frente) até a linha 2 ----
        etapaAtual = "Re ate linha 2";
        andarPolegadas(-POTENCIA_FRENTE, 0, DISTANCIA_RE);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 4) Ajuste final para dentro da loading zone F2 ----
        etapaAtual = "Entrando na loading zone F2";
        andarPolegadas(0, POTENCIA_LADO, DISTANCIA_LADO_2);

        // ---- 5) ESTACIONADO ----
        etapaAtual = "Estacionado";
        pararTudo();
        while (opModeIsActive()) {
            mostrar(etapaAtual);
        }
    }

    // ============ FUNÇÕES ============

    // Anda uma distância em polegadas convertendo para tempo com a calibração.
    // frente > 0 = para frente | lado > 0 = para a direita
    void andarPolegadas(double frente, double lado, double polegadas) {
        double velocidade = (lado != 0) ? POL_POR_SEG_LADO : POL_POR_SEG_FRENTE;
        andar(frente, lado, polegadas / velocidade);
    }

    // Anda por tempo mantendo a direção com o IMU
    void andar(double frente, double lado, double segundos) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < segundos && tempoTotal.seconds() < TEMPO_LIMITE) {
            double erro = normalizar(alvoHeading - headingGraus());
            double correcao = -erro * KP_RETO;
            mover(frente, lado, correcao);
            mostrar(etapaAtual);
        }
        mover(0, 0, 0);
    }

    // Gira até o ângulo pedido usando o IMU (+ esquerda, - direita)
    void girar(double graus) {
        alvoHeading = normalizar(alvoHeading + graus);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 3.0 && tempoTotal.seconds() < TEMPO_LIMITE) {
            double erro = normalizar(alvoHeading - headingGraus());
            if (Math.abs(erro) < GIRO_TOLERANCIA) break;

            double potencia = Math.abs(erro) * KP_GIRO;
            potencia = Math.max(GIRO_POTENCIA_MIN, Math.min(GIRO_POTENCIA_MAX, potencia));
            mover(0, 0, erro > 0 ? -potencia : potencia);
            mostrar("Girando");
        }
        mover(0, 0, 0);
    }

    // Modo calibração: 1 s para frente, pausa de 3 s para medir, 1 s para a direita
    void calibrar() {
        etapaAtual = "CALIBRANDO: 1 s para frente";
        andar(POTENCIA_FRENTE, 0, 1.0);
        etapaAtual = "MECA a distancia para frente";
        esperar(3.0);
        etapaAtual = "CALIBRANDO: 1 s para a direita";
        andar(0, POTENCIA_LADO, 1.0);
        etapaAtual = "MECA a distancia de lado";
        while (opModeIsActive()) {
            mostrar(etapaAtual);
        }
    }

    void soltarUmaBola() {
        servo.setPosition(PORTAO_ABERTO);
        esperar(TEMPO_PORTAO_ABERTO);
        servo.setPosition(PORTAO_FECHADO);
    }

    void esperar(double segundos) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < segundos) {
            mostrar(etapaAtual);
        }
    }

    // Mesma conta de mecanum do TeleOp (robot-centric)
    void mover(double y, double x, double rx) {
        if (INVERTER_STRAFE) x = -x;
        x = x * 1.1;
        double denominador = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominador);
        backLeft.setPower((y - x + rx) / denominador);
        frontRight.setPower((y - x - rx) / denominador);
        backRight.setPower((y + x - rx) / denominador);
    }

    void pararTudo() {
        mover(0, 0, 0);
        shooter.setPower(0);
        intake.setPower(0);
    }

    double headingGraus() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    double normalizar(double graus) {
        while (graus > 180)  graus -= 360;
        while (graus < -180) graus += 360;
        return graus;
    }

    void mostrar(String etapa) {
        telemetry.addData("Etapa", etapa);
        telemetry.addData("Tempo", "%.1f / 30 s", tempoTotal.seconds());
        telemetry.addData("Bolas soltas", "%d / %d", bolasSoltas, QUANTIDADE_BOLAS);
        telemetry.addData("Heading", "%.1f", headingGraus());
        telemetry.addData("Alvo", "%.1f", alvoHeading);
        telemetry.update();
    }
}