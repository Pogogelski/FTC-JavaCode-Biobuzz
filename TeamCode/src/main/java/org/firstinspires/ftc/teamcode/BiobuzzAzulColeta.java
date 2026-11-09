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
 * AUTÔNOMO ALIANÇA AZUL — D6 -> coleta no F6 -> volta ao D6
 *
 * A arena é simétrica por rotação de 180° em torno do centro (72, 72).
 * Girando o percurso vermelho (C1 -> A1 -> C1) 180°, o azul fica assim (polegadas):
 *
 *   Início:  tile D6  -> centro do robô em (84, 132)
 *            FRENTE do robô (coletor) virada para a parede do FUNDO (y = 144)
 *            COSTAS do robô (shooter) viradas para a HIVE
 *
 *   Com o robô olhando para o fundo (+y):
 *     ir para a plateia (-y)           = robô anda de RÉ
 *     ir para o fundo (+y)             = robô anda para FRENTE
 *     ir para a direita da arena (+x)  = robô anda de lado para a DIREITA dele
 *
 *   O GARDEN AZUL fica encostado na parede do fundo, em x 121–144 (tile F6).
 *   Como o coletor já está virado para essa parede, o robô só precisa andar
 *   de lado até F6 e avançar com o intake ligado.
 *
 *   Passo 1: lança 3 bolas parado em D6
 *   Passo 2: RÉ 6 pol         -> y = 126 (afasta da parede para não chutar as bolas do garden)
 *   Passo 3: lado DIREITA 48  -> x = 132 (tile F6, na frente do garden)
 *   Passo 4: intake ligado, avança devagar até o garden e espera coletar
 *   Passo 5: RÉ 6 pol         -> sai do garden
 *   Passo 6: lado ESQUERDA 48 -> x = 84 (volta para a coluna D)
 *   Passo 7: FRENTE 6 pol     -> y = 132 (mesmo ponto de lançamento do início)
 *   Passo 8: lança 4 bolas
 *   Fim:     tudo antes de 30 s (se o tempo acabar, ele para de lançar)
 *
 * Como é uma rotação, os comandos do robô são IGUAIS aos do vermelho com coleta;
 * só muda onde ele é colocado no campo.
 *
 * ORÇAMENTO DE TEMPO (com os valores abaixo e a calibração de exemplo):
 *   lançar 3 ≈ 6,6 s | ida ≈ 3,9 s | coleta ≈ 1,8 s | volta ≈ 4,5 s
 *   lançar 4 ≈ 7,0 s  -> total ≈ 24 s
 *   Com os tempos antigos (acelerar 5 s, intervalo 3,5 s) passaria de 40 s,
 *   por isso o shooter fica LIGADO o tempo todo e o intervalo caiu para 1,5 s.
 */
@Autonomous(name = "Biobuzz-azul-coleta", preselectTeleOp = "Biobuzz2026")
public class BiobuzzAzulColeta extends LinearOpMode {

    // ============ MESMOS AJUSTES DO TELEOP ============
    static final boolean INVERTER_STRAFE = true;

    // ============ TEMPO ============
    static final double TEMPO_LIMITE = 29.0;

    // ============ SHOOTER E PORTÃO ============
    static final double SHOOTER_AUTONOMO = -0.7;
    // true = shooter não desliga durante a ida e volta (economiza a 2ª aceleração).
    static final boolean MANTER_SHOOTER_LIGADO = true;
    static final double TEMPO_ACELERAR    = 3.0;  // 1ª aceleração, antes da 1ª bola
    static final double TEMPO_REACELERAR  = 3.0;  // só usado se MANTER_SHOOTER_LIGADO = false
    static final double TEMPO_ESTABILIZAR = 0.5;  // espera ao chegar em D6 antes da 1ª bola da 2ª rodada
    static final double TEMPO_DESLIGAR    = 1.2;  // shooter ainda gira isso depois da última bola
    static final int    BOLAS_RODADA_1    = 3;
    static final int    BOLAS_RODADA_2    = 4;
    // Tempo para o shooter recuperar a rotação entre uma bola e outra.
    // Se a 2ª/3ª bola sair fraca, aumente (mas cuidado com os 30 s).
    static final double INTERVALO_BOLAS   = 1.5;
    // Se NENHUMA bola cair, aumente (ex.: 0.25). Se caírem DUAS, diminua (ex.: 0.10).
    static final double TEMPO_PORTAO_ABERTO = 0.20;

    static final double SERVO_CURSO_TOTAL = 270.0;
    static final double SERVO_LIMITE      = 90.0;
    static final double SERVO_ANGULO      = 50.0;
    static final double PORTAO_ABERTO     = 0.0;
    static final double PORTAO_FECHADO    = SERVO_ANGULO / SERVO_LIMITE;

    // ============ INTAKE ============
    // Se o intake girar para o lado errado (cuspindo em vez de puxar), troque o sinal.
    static final double INTAKE_COLETAR     = 1.0;
    static final double POTENCIA_COLETA    = 0.25; // avanço devagar contra o garden
    static final double TEMPO_EMPURRAR     = 0.8;  // quanto tempo avança com o intake ligado
    static final double TEMPO_COLETA       = 1.0;  // parado no garden com o intake ligado

    // ============ PERCURSO (em polegadas) ============
    static final double DISTANCIA_AFASTAR = 6.0;  // D6 -> 6 pol para longe da parede
    // D (x=84) -> F (x=132): 2 tiles.
    // Se não ficar alinhado com o garden, ajuste aqui.
    static final double DISTANCIA_LADO    = 48.0;

    static final double POTENCIA_FRENTE = 0.5;
    static final double POTENCIA_LADO   = 0.5;

    // ============ CALIBRAÇÃO (IMPORTANTE!) ============
    static final boolean CALIBRAR = false;
    static final double POL_POR_SEG_FRENTE = 20.0; // valor de exemplo, MEÇA o seu
    static final double POL_POR_SEG_LADO   = 16.0;

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

        telemetry.addLine(CALIBRAR ? "MODO CALIBRACAO" : "AZUL COLETA: robo em D6, coletor para o fundo, shooter para a HIVE.");
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
        // ========= SEQUÊNCIA DO AUTÔNOMO AZUL COLETA =========
        // =====================================================

        // ---- 1) LANÇAR 3 BOLAS (parado em D6) ----
        etapaAtual = "Acelerando shooter";
        servo.setPosition(PORTAO_FECHADO);
        shooter.setPower(SHOOTER_AUTONOMO);
        esperar(TEMPO_ACELERAR);
        lancar(BOLAS_RODADA_1);

        if (!MANTER_SHOOTER_LIGADO) {
            etapaAtual = "Ultima bola saindo";
            esperar(TEMPO_DESLIGAR);
            shooter.setPower(0);
        }

        // ---- 2) Afasta da parede: RÉ 6 pol ----
        etapaAtual = "Re para afastar da parede";
        andarPolegadas(-POTENCIA_FRENTE, 0, DISTANCIA_AFASTAR);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 3) D -> F: lado para a DIREITA do robô (= direita da arena) ----
        etapaAtual = "Lado ate F6";
        andarPolegadas(0, POTENCIA_LADO, DISTANCIA_LADO);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 4) COLETA: intake ligado, avança devagar até o garden ----
        etapaAtual = "Coletando";
        intake.setPower(INTAKE_COLETAR);
        andar(POTENCIA_COLETA, 0, TEMPO_EMPURRAR);
        esperar(TEMPO_COLETA);

        // ---- 5) Sai do garden: RÉ 6 pol (intake continua puxando) ----
        etapaAtual = "Re saindo do garden";
        andarPolegadas(-POTENCIA_FRENTE, 0, DISTANCIA_AFASTAR);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 6) F -> D: lado para a ESQUERDA do robô (= esquerda da arena) ----
        etapaAtual = "Lado de volta para D6";
        andarPolegadas(0, -POTENCIA_LADO, DISTANCIA_LADO);
        intake.setPower(0);
        esperar(PAUSA_ENTRE_MOVIMENTOS);

        // ---- 7) Volta ao ponto de lançamento: FRENTE 6 pol ----
        etapaAtual = "Frente ate D6";
        andarPolegadas(POTENCIA_FRENTE, 0, DISTANCIA_AFASTAR);

        // ---- 8) LANÇAR 4 BOLAS ----
        if (MANTER_SHOOTER_LIGADO) {
            etapaAtual = "Estabilizando";
            esperar(TEMPO_ESTABILIZAR);
        } else {
            etapaAtual = "Reacelerando shooter";
            shooter.setPower(SHOOTER_AUTONOMO);
            esperar(TEMPO_REACELERAR);
        }
        lancar(BOLAS_RODADA_2);

        etapaAtual = "Ultima bola saindo";
        esperar(TEMPO_DESLIGAR);

        // ---- 9) FIM ----
        etapaAtual = "Fim";
        pararTudo();
        while (opModeIsActive()) {
            mostrar(etapaAtual);
        }
    }

    // ============ FUNÇÕES ============

    // Solta 'quantidade' bolas com o shooter já girando.
    // Se o tempo do autônomo estiver acabando, para de lançar.
    void lancar(int quantidade) {
        for (int i = 1; i <= quantidade && opModeIsActive(); i++) {
            if (i > 1) {
                etapaAtual = "Intervalo entre bolas";
                esperar(INTERVALO_BOLAS);
            }
            if (tempoTotal.seconds() > TEMPO_LIMITE - TEMPO_PORTAO_ABERTO) break;
            etapaAtual = "Soltando bola " + i + " de " + quantidade;
            soltarUmaBola();
            bolasSoltas++;
        }
    }

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
        telemetry.addData("Bolas soltas", "%d / %d", bolasSoltas, BOLAS_RODADA_1 + BOLAS_RODADA_2);
        telemetry.addData("Heading", "%.1f", headingGraus());
        telemetry.addData("Alvo", "%.1f", alvoHeading);
        telemetry.update();
    }
}