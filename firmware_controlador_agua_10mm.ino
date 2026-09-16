/* =============================================================================
   FIRMWARE - CONTROLADOR_AGUA_10MM
   Valvula borboleta rotativa com corte automatico por vazamento
   -----------------------------------------------------------------------------
   Escrito a partir da geometria do modelo CAD enviado:

     corpo.step          88 x 50 x 36 mm, canal de passagem Ø10 mm no eixo X
                         rebaixos Ø10,3 nas pontas (assento de o-ring / tubo)
                         camara do obturador Ø32, bucha do eixo Ø6,36
                         fixacao 4x M4 em (+-24, +-17)
     obturador.step      disco Ø32 x 3 mm, perpendicular ao fluxo = FECHADO
                         furo central Ø6 para o eixo
     eixo.step           Ø6 mm vertical (Z), de z=-19 ate z=+24
                         ponta superior dentro da caixa eletronica
     placa_sensores.step PCB 54 x 34 x 3 mm em z=20..23, furos M4 em (+-20,+-12)
                         centrada sobre a ponta do eixo -> sensor de angulo
     caixa_eletronica    68 x 48 x 22 mm em z=17..39
                         3 passagens Ø7 na parede +Y em x = -20 / 0 / +20
     tampa + junta       vedacao da caixa em z=37..43

   CONSEQUENCIAS PARA O CONTROLE
     - O curso e ROTATIVO de 90 graus: 0 = fechado, 90 = aberto.
       Nao existem fins de curso; a posicao vem do encoder magnetico AS5600
       lendo um ima diametral colado na ponta do eixo (z=24), logo abaixo
       do chip no centro da placa_sensores.
     - Sem realimentacao de corrente: o travamento (stall) e detectado pelo
       proprio encoder - motor acionado e angulo parado = obstrucao.
     - As 3 passagens Ø7 definem a fiacao externa:
          x=-20  alimentacao 12 V
          x=  0  sensor de vazamento externo (sonda/alagamento)
          x=+20  comando remoto / sensor de fluxo (opcional)

   LOGICA DE SEGURANCA
     Vazamento confirmado com a valvula aberta -> contagem de 60 s ->
     fechamento automatico + bloqueio persistente (EEPROM) + sirene.
     Se o vazamento cessar dentro dos 60 s, a contagem e cancelada.

   Plataforma: Arduino (AVR) ou ESP32 / ESP32-C3. I2C para o AS5600.
   ============================================================================= */

#include <Wire.h>
#include <EEPROM.h>

/* =============================================================================
   1) TIPO DE ATUADOR NO EIXO Ø6
   -----------------------------------------------------------------------------
   ATU_MOTOR_DC : micromotor com redutor (N20 ou similar) + ponte H DRV8833/L9110
                  acoplado ao rabicho achatado do eixo. Malha fechada pelo AS5600.
                  Mantem posicao sem energia. Recomendado para este projeto.
   ATU_SERVO    : servo RC metalico (>= 10 kgf.cm) com acoplador no eixo.
                  Mais simples, porem consome energia parado e nao e fail-safe.
   ============================================================================= */
#define ATU_MOTOR_DC 1
#define ATU_SERVO    2

#define ATUADOR   ATU_MOTOR_DC        // <<< selecione aqui

#if ATUADOR == ATU_SERVO
  #include <Servo.h>
  Servo servoValvula;
#endif

/* =============================================================================
   2) PINAGEM  (ajuste conforme a sua placa de 54 x 34 mm)
   ============================================================================= */
#if defined(ESP32)
  const uint8_t PIN_SDA         = 8;
  const uint8_t PIN_SCL         = 9;
  const uint8_t PIN_MOT_A       = 3;    // ponte H IN1 (PWM)
  const uint8_t PIN_MOT_B       = 4;    // ponte H IN2 (PWM)
  const uint8_t PIN_SONDA_A     = 0;    // eletrodo de conducao (ADC)
  const uint8_t PIN_SONDA_EXC   = 1;    // excitacao alternada do eletrodo
  const uint8_t PIN_VAZ_EXT     = 5;    // sensor de alagamento externo (digital)
  const uint8_t PIN_FLUXO       = 6;    // sensor de fluxo opcional (pulsos)
  const uint8_t PIN_BTN_CMD     = 7;    // botao / canal do receptor RF
  const uint8_t PIN_BTN_RESET   = 10;   // rearme (segurar 3 s)
  const uint8_t PIN_BUZZER      = 2;
  const uint8_t PIN_LED_VERDE   = 20;
  const uint8_t PIN_LED_VERM    = 21;
#else   // Arduino Nano / Pro Mini
  const uint8_t PIN_MOT_A       = 5;    // PWM
  const uint8_t PIN_MOT_B       = 6;    // PWM
  const uint8_t PIN_SONDA_A     = A0;
  const uint8_t PIN_SONDA_EXC   = 7;
  const uint8_t PIN_VAZ_EXT     = 2;
  const uint8_t PIN_FLUXO       = 3;    // INT1
  const uint8_t PIN_BTN_CMD     = 4;
  const uint8_t PIN_BTN_RESET   = 8;
  const uint8_t PIN_BUZZER      = 9;
  const uint8_t PIN_LED_VERDE   = 10;
  const uint8_t PIN_LED_VERM    = 11;
#endif

/* =============================================================================
   3) PARAMETROS DO MECANISMO (vindos do CAD)
   ============================================================================= */
const float ANG_FECHADO   =  0.0f;    // disco perpendicular ao fluxo
const float ANG_ABERTO    = 90.0f;    // disco alinhado ao fluxo
const float TOLERANCIA    =  2.5f;    // graus - janela de "chegou"
const float ANG_DESACEL   = 20.0f;    // graus - inicio da rampa de frenagem

const uint8_t PWM_MIN     =  90;      // minimo para vencer o atrito da bucha Ø6
const uint8_t PWM_MAX     = 255;

/* =============================================================================
   4) TEMPOS (ms)
   ============================================================================= */
const unsigned long T_CORTE_VAZAMENTO = 60000UL;  // 1 MINUTO ate fechar sozinha
const unsigned long T_DEBOUNCE_SONDA  =  2000UL;  // sensor estavel por 2 s
const unsigned long T_MANOBRA_MAX     =  8000UL;  // 90 graus devem sair bem antes
const unsigned long T_STALL           =  1200UL;  // sem movimento = travado
const unsigned long T_SEGURAR_RESET   =  3000UL;
const unsigned long T_DEBOUNCE_BOTAO  =    50UL;

/* Deteccao por fluxo continuo (opcional). Zere para desabilitar. */
const bool          USAR_SENSOR_FLUXO = false;
const unsigned long T_FLUXO_CONTINUO  = 1800000UL; // 30 min de fluxo ininterrupto
const float         PULSOS_POR_LITRO  = 450.0f;    // YF-S201 em linha de Ø10

/* Limiar da sonda de conducao: quanto MENOR a leitura, mais condutivo/molhado */
const int LIMIAR_SONDA_MOLHADA = 400;   // 0..1023 (AVR) / reescale no ESP32

/* =============================================================================
   5) ENCODER MAGNETICO AS5600
   ============================================================================= */
const uint8_t AS5600_ADDR    = 0x36;
const uint8_t AS5600_RAWANG  = 0x0C;   // 12 bits, 0..4095
const uint8_t AS5600_STATUS  = 0x0B;   // MD/ML/MH - presenca do ima

bool as5600Ler(uint16_t &bruto) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAWANG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = Wire.read(), lo = Wire.read();
  bruto = ((hi << 8) | lo) & 0x0FFF;
  return true;
}

bool as5600ImaOk() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return false;
  uint8_t st = Wire.read();
  return (st & 0x20);            // MD = ima detectado
}

/* =============================================================================
   6) DADOS PERSISTENTES
   ============================================================================= */
struct Config {
  uint16_t magic;
  uint16_t zeroBruto;    // leitura do encoder na posicao FECHADA
  int8_t   sentido;      // +1 ou -1: sentido de crescimento do angulo
  uint8_t  bloqueado;    // trava apos corte por vazamento
};
const uint16_t MAGIC = 0xC0A1;
Config cfg;

void salvarCfg() {
  EEPROM.put(0, cfg);
#if defined(ESP32)
  EEPROM.commit();
#endif
}

void carregarCfg() {
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC) {            // primeira energizacao
    cfg.magic     = MAGIC;
    cfg.zeroBruto = 0;
    cfg.sentido   = 1;
    cfg.bloqueado = 0;
    salvarCfg();
  }
}

/* =============================================================================
   7) ESTADOS
   ============================================================================= */
enum Estado : uint8_t {
  ST_FECHADA, ST_ABRINDO, ST_ABERTA, ST_PRE_ALARME,
  ST_FECHANDO, ST_BLOQUEADA, ST_FALHA
};
Estado estado = ST_FECHADA;

unsigned long tEstado = 0, tSondaMudou = 0, tReset = 0;
unsigned long tRefStall = 0, tFluxoInicio = 0;
float  anguloAtual = 0.0f, anguloRefStall = 0.0f, alvo = ANG_FECHADO;
bool   leituraBruta = false, vazamentoConfirmado = false;
uint8_t tentativas = 0;

volatile unsigned long pulsosFluxo = 0;
void ISR_fluxo() { pulsosFluxo++; }

void trocarEstado(Estado n) { estado = n; tEstado = millis(); }

/* =============================================================================
   8) LEITURA DE POSICAO
   ============================================================================= */
float lerAngulo() {
  uint16_t bruto;
  if (!as5600Ler(bruto)) return anguloAtual;         // mantem o ultimo valor
  int16_t delta = (int16_t)bruto - (int16_t)cfg.zeroBruto;
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return cfg.sentido * delta * (360.0f / 4096.0f);
}

/* =============================================================================
   9) ACIONAMENTO
   ============================================================================= */
void motorParar() {
#if ATUADOR == ATU_MOTOR_DC
  analogWrite(PIN_MOT_A, 0);
  analogWrite(PIN_MOT_B, 0);
#endif
}

/* pwm > 0 abre (sentido de angulo crescente), pwm < 0 fecha */
void motorAcionar(int pwm) {
#if ATUADOR == ATU_MOTOR_DC
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) { analogWrite(PIN_MOT_B, 0); analogWrite(PIN_MOT_A, pwm); }
  else if (pwm < 0) { analogWrite(PIN_MOT_A, 0); analogWrite(PIN_MOT_B, -pwm); }
  else motorParar();
#endif
}

/* Perfil de velocidade: PWM cheio longe do alvo, rampa nos ultimos graus.
   Evita batida do disco Ø32 contra a parede da camara. */
void irParaAlvo(float destino) {
#if ATUADOR == ATU_SERVO
  servoValvula.write((int)constrain(destino, 0, 180));
#else
  float erro = destino - anguloAtual;
  float mod  = fabs(erro);
  if (mod <= TOLERANCIA) { motorParar(); return; }
  float k = (mod >= ANG_DESACEL) ? 1.0f : (mod / ANG_DESACEL);
  int pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * k);
  motorAcionar(erro > 0 ? pwm : -pwm);
#endif
}

bool chegou(float destino) { return fabs(destino - anguloAtual) <= TOLERANCIA; }

/* Travamento: motor comandado e angulo praticamente parado por T_STALL */
bool detectouStall() {
#if ATUADOR == ATU_SERVO
  return false;
#else
  unsigned long agora = millis();
  if (fabs(anguloAtual - anguloRefStall) > 1.0f) {
    anguloRefStall = anguloAtual;
    tRefStall = agora;
    return false;
  }
  return (agora - tRefStall >= T_STALL);
#endif
}

void iniciarManobra(float destino) {
  alvo = destino;
  anguloRefStall = anguloAtual;
  tRefStall = millis();
}

/* =============================================================================
   10) DETECCAO DE VAZAMENTO
   -----------------------------------------------------------------------------
   Duas entradas independentes do controle de posicao:
     a) sonda de conducao na bandeja / sob o corpo (excitacao alternada para
        nao eletrolisar os eletrodos);
     b) contato seco de um sensor de alagamento externo, pelo prensa-cabo x=0.
   Opcionalmente, fluxo continuo por 30 min tambem caracteriza vazamento.
   ============================================================================= */
bool lerSondaConducao() {
  digitalWrite(PIN_SONDA_EXC, HIGH);
  delayMicroseconds(200);
  int v1 = analogRead(PIN_SONDA_A);
  digitalWrite(PIN_SONDA_EXC, LOW);      // inverte para nao polarizar
  delayMicroseconds(200);
  return (v1 < LIMIAR_SONDA_MOLHADA);
}

bool lerFluxoContinuo() {
  if (!USAR_SENSOR_FLUXO) return false;
  static unsigned long ultimaJanela = 0, ultimoTotal = 0;
  unsigned long agora = millis();
  if (agora - ultimaJanela < 1000UL) return (tFluxoInicio &&
                                             agora - tFluxoInicio >= T_FLUXO_CONTINUO);
  ultimaJanela = agora;
  noInterrupts(); unsigned long total = pulsosFluxo; interrupts();
  bool temFluxo = (total - ultimoTotal) > 3;    // ~3 pulsos/s de ruido
  ultimoTotal = total;
  if (temFluxo) { if (!tFluxoInicio) tFluxoInicio = agora; }
  else tFluxoInicio = 0;
  return (tFluxoInicio && agora - tFluxoInicio >= T_FLUXO_CONTINUO);
}

/* =============================================================================
   11) SINALIZACAO
   ============================================================================= */
void sinalizar() {
  unsigned long ms = millis();
  switch (estado) {
    case ST_ABERTA:
      digitalWrite(PIN_LED_VERDE, HIGH); digitalWrite(PIN_LED_VERM, LOW);
      noTone(PIN_BUZZER); break;

    case ST_PRE_ALARME: {
      bool p = (ms / 250) % 2;
      digitalWrite(PIN_LED_VERDE, HIGH); digitalWrite(PIN_LED_VERM, p);
      if (p) tone(PIN_BUZZER, 2000); else noTone(PIN_BUZZER);
      break;
    }
    case ST_BLOQUEADA:
      digitalWrite(PIN_LED_VERDE, LOW); digitalWrite(PIN_LED_VERM, HIGH);
      if (ms - tEstado < 30000UL) tone(PIN_BUZZER, 2500); else noTone(PIN_BUZZER);
      break;

    case ST_FALHA: {
      bool p = (ms / 800) % 2;
      digitalWrite(PIN_LED_VERDE, LOW); digitalWrite(PIN_LED_VERM, p);
      noTone(PIN_BUZZER); break;
    }
    default:
      digitalWrite(PIN_LED_VERDE,
        (estado == ST_ABRINDO || estado == ST_FECHANDO) ? ((ms / 400) % 2) : LOW);
      digitalWrite(PIN_LED_VERM, LOW);
      noTone(PIN_BUZZER); break;
  }
}

/* =============================================================================
   12) BOTOES E CALIBRACAO
   ============================================================================= */
bool botao(uint8_t p) {
  if (digitalRead(p) == LOW) { delay(T_DEBOUNCE_BOTAO); return digitalRead(p) == LOW; }
  return false;
}

void tratarRearme() {
  if (digitalRead(PIN_BTN_RESET) == LOW) {
    if (!tReset) tReset = millis();
    if (millis() - tReset >= T_SEGURAR_RESET) {
      if (vazamentoConfirmado) {
        tone(PIN_BUZZER, 400, 600);
        Serial.println(F("REARME NEGADO - sensor ainda acusa vazamento."));
      } else {
        cfg.bloqueado = 0; salvarCfg();
        tone(PIN_BUZZER, 1500, 200);
        Serial.println(F("REARME OK."));
        tentativas = 0;
        trocarEstado(ST_FECHADA);
      }
      tReset = 0;
    }
  } else tReset = 0;
}

/* Console serie:
     Z  -> grava a posicao atual como FECHADO (0 grau)
     I  -> inverte o sentido de leitura do encoder
     A  -> comanda abrir     F -> comanda fechar     ?  -> status           */
void tratarConsole() {
  if (!Serial.available()) return;
  char c = toupper(Serial.read());
  uint16_t bruto;
  switch (c) {
    case 'Z':
      if (as5600Ler(bruto)) { cfg.zeroBruto = bruto; salvarCfg();
        Serial.println(F("Zero gravado na posicao atual.")); }
      break;
    case 'I':
      cfg.sentido = -cfg.sentido; salvarCfg();
      Serial.print(F("Sentido = ")); Serial.println(cfg.sentido);
      break;
    case 'A':
      if (estado == ST_FECHADA && !vazamentoConfirmado) {
        iniciarManobra(ANG_ABERTO); trocarEstado(ST_ABRINDO); }
      break;
    case 'F':
      if (estado == ST_ABERTA || estado == ST_PRE_ALARME) {
        iniciarManobra(ANG_FECHADO); trocarEstado(ST_FECHANDO); }
      break;
    case '?':
      Serial.print(F("estado=")); Serial.print(estado);
      Serial.print(F("  ang=")); Serial.print(anguloAtual, 1);
      Serial.print(F("  vaz=")); Serial.print(vazamentoConfirmado);
      Serial.print(F("  bloq=")); Serial.println(cfg.bloqueado);
      break;
  }
}

/* =============================================================================
   13) SETUP
   ============================================================================= */
void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  EEPROM.begin(sizeof(Config) + 8);
  Wire.begin(PIN_SDA, PIN_SCL);
#else
  Wire.begin();
#endif
  Wire.setClock(400000);
  carregarCfg();

  pinMode(PIN_MOT_A, OUTPUT); pinMode(PIN_MOT_B, OUTPUT);
  motorParar();

  pinMode(PIN_SONDA_EXC, OUTPUT); digitalWrite(PIN_SONDA_EXC, LOW);
  pinMode(PIN_VAZ_EXT,   INPUT_PULLUP);
  pinMode(PIN_BTN_CMD,   INPUT_PULLUP);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_LED_VERDE,  OUTPUT);
  pinMode(PIN_LED_VERM,   OUTPUT);

  if (USAR_SENSOR_FLUXO) {
    pinMode(PIN_FLUXO, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLUXO), ISR_fluxo, FALLING);
  }

#if ATUADOR == ATU_SERVO
  servoValvula.attach(PIN_MOT_A);
#endif

  if (!as5600ImaOk()) {
    Serial.println(F("ATENCAO: ima do eixo nao detectado pelo AS5600."));
    Serial.println(F("Verifique o ima diametral na ponta do eixo (z=24)."));
  }

  anguloAtual = lerAngulo();

  // Partida sempre segura: fecha antes de qualquer coisa.
  iniciarManobra(ANG_FECHADO);
  trocarEstado(ST_FECHANDO);
  Serial.println(F("Controlador_Agua_10mm iniciado - fechando por seguranca."));
}

/* =============================================================================
   14) LOOP
   ============================================================================= */
void loop() {
  unsigned long agora = millis();

  /* -- posicao -- */
  anguloAtual = lerAngulo();

  /* -- vazamento, com filtro de 2 s -- */
  bool bruto = lerSondaConducao() || (digitalRead(PIN_VAZ_EXT) == LOW)
               || lerFluxoContinuo();
  if (bruto != leituraBruta) { leituraBruta = bruto; tSondaMudou = agora; }
  if (agora - tSondaMudou >= T_DEBOUNCE_SONDA) vazamentoConfirmado = leituraBruta;

  /* -- maquina de estados -- */
  switch (estado) {

    case ST_FECHADA:
      motorParar();
      if (botao(PIN_BTN_CMD)) {
        if (vazamentoConfirmado) {
          Serial.println(F("Abertura negada: vazamento ativo."));
          tone(PIN_BUZZER, 400, 600);
        } else {
          iniciarManobra(ANG_ABERTO);
          trocarEstado(ST_ABRINDO);
        }
      }
      break;

    case ST_ABRINDO:
      irParaAlvo(ANG_ABERTO);
      if (vazamentoConfirmado) {            // aborta e fecha na hora
        Serial.println(F("Vazamento durante a abertura - fechando."));
        cfg.bloqueado = 1; salvarCfg();
        iniciarManobra(ANG_FECHADO); trocarEstado(ST_FECHANDO);
      }
      else if (chegou(ANG_ABERTO)) { motorParar(); tentativas = 0; trocarEstado(ST_ABERTA); }
      else if (detectouStall() || agora - tEstado > T_MANOBRA_MAX) {
        motorParar();
        Serial.println(F("Obstrucao na abertura."));
        iniciarManobra(ANG_FECHADO); trocarEstado(ST_FECHANDO);
      }
      break;

    case ST_ABERTA:
      motorParar();
      if (vazamentoConfirmado) {
        Serial.println(F("VAZAMENTO - fechamento automatico em 60 s."));
        trocarEstado(ST_PRE_ALARME);
      } else if (botao(PIN_BTN_CMD)) {
        iniciarManobra(ANG_FECHADO); trocarEstado(ST_FECHANDO);
      }
      break;

    case ST_PRE_ALARME:
      motorParar();
      if (!vazamentoConfirmado) {
        Serial.println(F("Vazamento cessou - contagem cancelada."));
        trocarEstado(ST_ABERTA);
      }
      else if (botao(PIN_BTN_CMD) || agora - tEstado >= T_CORTE_VAZAMENTO) {
        Serial.println(F("Fechando e bloqueando."));
        cfg.bloqueado = 1; salvarCfg();
        iniciarManobra(ANG_FECHADO); trocarEstado(ST_FECHANDO);
      }
      break;

    case ST_FECHANDO:
      irParaAlvo(ANG_FECHADO);
      if (chegou(ANG_FECHADO)) {
        motorParar(); tentativas = 0;
        trocarEstado(cfg.bloqueado ? ST_BLOQUEADA : ST_FECHADA);
      }
      else if (detectouStall() || agora - tEstado > T_MANOBRA_MAX) {
        motorParar();
        // fechar e critico: tenta desencravar recuando 15 graus e insistindo
        if (++tentativas <= 2) {
          Serial.println(F("Travou ao fechar - tentando desencravar."));
          motorAcionar(PWM_MAX); delay(250); motorParar();
          iniciarManobra(ANG_FECHADO); tEstado = agora;
        } else {
          Serial.println(F("FALHA: nao foi possivel fechar."));
          cfg.bloqueado = 1; salvarCfg();
          trocarEstado(ST_FALHA);
        }
      }
      break;

    case ST_BLOQUEADA:
    case ST_FALHA:
      motorParar();
      tratarRearme();
      break;
  }

  sinalizar();
  tratarConsole();
}
