#include <I2C.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comente para restringir os valores entre ±90°  http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Instâncias do filtro kalman
Kalman kalmanY;

/* dados da MPU6050 */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Ângulo calculado usando somente o giroscópio
double compAngleX, compAngleY; // Ângulo calculado usando o filtro complementar
double kalAngleX, kalAngleY; // Ângulo calculado usando o filtro Kalman

uint32_t timer;
uint8_t i2cData[14]; // Buffer para o I2C

int vcc_MPU6050 = 13; //PWM para alimentar os botões
int botao_L = 4; //Saída digital para o botão esquerdo
int botao_R = 7; //Saída digital para o botão direito
int estado_L = 0; //indicadores de estado (apertar e soltar)
int estado_R = 0;

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();

  analogWrite(vcc_MPU6050, 255); //saída pwm sempre ligada

  TWBR = ((F_CPU / 400000L) - 16) / 2; // Frequência do I2C 400kHz

  i2cData[0] = 7; // Definir a taxa de amostragem de 1000Hz - 8kHz / (7 + 1) = 1000Hz
  i2cData[1] = 0x00; // Desativar FSYNC e definir filtragem Acc a 260Hz, filtragem de 256Hz Giroscópio, amostragem de 8KHz
  i2cData[2] = 0x00; // Escala completa do giroscópio ±250graus/s
  i2cData[3] = 0x00; // Acelerômetro no máximo, ±2g (g = aceleração gravitacional)
  while (i2cWrite(0x19, i2cData, 4, false)); // Escrever nos registradores
  while (i2cWrite(0x6B, 0x01, true)); // PLL com referência no eixo X do giroscópio e desativar o modo de suspensão

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Registrador de endereço da MPU6050 (a 5v)
    Serial.print(F("Erro de leitura"));
    while (1);
  }

  pinMode(botao_L, INPUT);
  delay(100); // Delay para estabilização do sensor

  /* Setar valores iniciais dos algulos Kalman */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 e eq. 26
  // atan2 converte os valores de π (pi) para radianos
  // Para converter radianos em graus
#ifdef RESTRICT_PITCH // Eq. 25 e 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Setar ângulo inicial
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void leitura() {
  /* atualizar todos os valores */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // Concertar problema de transição quando o acelrômetro pula de -180 para 180 graus.
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculo do ângulo utilizado o filtro Kalman

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Se a taxa for invertida, então a leitura do acelerômetro fica restrita.
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // Concertar problema de transição quando o acelrômetro pula de -180 para 180 graus.
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculo do ângulo utilizado o filtro Kalman

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Se a taxa for invertida, então a leitura do acelerômetro fica restrita.
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculo do ângulo utilizado o filtro Kalman
#endif

  gyroXangle += gyroXrate * dt; // Calculo do giroscópio sem o filtro
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calcular o ângulo do giroscópio utilizando a taxa imparcial
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calcula o ângulo utilizando o filtro complementar
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reinicia o ângulo do giroscópio quando o ângulo estiver muito afastado
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

#if 1 // Set to 1 to activate
  Serial.print(kalAngleX); Serial.print(";");
  Serial.print(kalAngleY); Serial.print(";");
#endif

}

void loop() {


  if (estado_L == LOW && digitalRead(botao_L) == LOW) {// verificar se a entrada é LOW (interruptor ocupado) e se já estava ocupado
    leitura();
    Serial.print("segura_esquerda");
  }
  else if (estado_R == LOW && digitalRead(botao_R) == LOW) {// verificar se a entrada é LOW (interruptor ocupado) e se já estava ocupado
    leitura();
    Serial.print("segura_direita");
  }
  else {
    estado_L = digitalRead(botao_L);  // ler o valor de entrada
    if (estado_L == LOW) {         // verificar se a entrada é LOW (interruptor ocupado)
      Serial.print("segura_esquerda");
    } else {
      estado_R = digitalRead(botao_R);
      if (estado_R == LOW) {         // verificar se a entrada é LOW (interruptor ocupado)
        Serial.print("segura_direita");
      } else {
        leitura();
      }
    }
  }

  Serial.println("");
  delay(55);
  /*Observe que o delay entre o arduino e a aplicação tem uma diferença de 5 milissegundos
  isso serve para não haver "filas" de dados enviados do arduíno, por causa do delay de transmissão*/
}
