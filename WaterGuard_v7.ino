/* ============================================================
   WATER GUARD - v7
   Cambios frente a v6:
   - La probabilidad de evaporacion ahora aparece como una
     LECTURA propia (junto a nivel, temperatura, humedad),
     no mezclada entre los datos internos del IRH. Se muestra
     con numero y categoria en palabras: BAJA / MEDIA / ALTA.
   - OLED rediseñada para ser mas facil de leer de un vistazo:
     el estado del sistema y si hay anomalia activa se
     imprimen en palabras completas ("Estado: NORMAL",
     "Anomalia: SI/NO") en vez de codigos de una letra, y la
     evaporacion tiene su propia fila con numero + categoria.

   Cambios frente a v5 (se mantienen):
   - Probabilidad de evaporacion (0-100%), calculada
     combinando VARIAS variables (temp. agua, temp. ambiente,
     humedad, luminosidad), cada una con su propio peso y
     normalizada solo sobre los sensores que estan OK en ese
     momento (mismo patron que el IRH). Ninguna variable sola
     puede mover el numero por si misma.
   - Serial + log CSV: columna de evaporacion.
   ============================================================ */

#include <Wire.h>
#include <U8x8lib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>


/* ============================================================
   OLED
   ============================================================ */

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);


/* ============================================================
   PINES
   ============================================================ */

#define PIN_TRIG            2
#define PIN_ECHO            3
#define PIN_DS18B20         4
#define PIN_LED_VERDE       5
#define PIN_LED_AMARILLO    6
#define PIN_LED_ROJO        7
#define PIN_BUZZER          8
#define PIN_LUZ             9   /* KY-018 digital: si hay luz o no */
#define PIN_DHT             10
#define PIN_FOTORESISTENCIA A0  /* nivel de luminosidad analogico */


/* ============================================================
   SENSORES
   ============================================================ */

#define DHTTYPE DHT11

DHT dht(PIN_DHT, DHTTYPE);

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorAgua(&oneWire);


/* ============================================================
   VARIABLES DE SENSORES
   ============================================================ */

float temperaturaAgua     = NAN;
float temperaturaAmbiente = NAN;
float humedad             = NAN;
float distancia           = NAN;
float nivel               = NAN;
bool  luz                 = false;

/* Nivel de luminosidad en porcentaje (0-100), a partir de la
   fotoresistencia. 0 = totalmente oscuro, 100 = luz maxima
   detectable por el divisor de voltaje.

   OJO CABLEADO: esto asume un divisor de voltaje foto-
   resistencia + resistencia fija a A0. Segun como quede
   armado el divisor, mas luz puede dar un ADC mas ALTO o
   mas BAJO. Se deja INVERTIR_FOTORESISTENCIA como bandera
   para no tener que tocar el resto del codigo si la lectura
   sale al reves una vez conectada. */
float luminosidad = NAN;

const bool INVERTIR_FOTORESISTENCIA = false;


/* ============================================================
   SALUD DE SENSORES

   Un sensor NO se declara caido por una sola lectura mala:
   el DHT11 y el HC-SR04 fallan de vez en cuando y eso hacia
   que el estado brincara sin razon.
   ============================================================ */

bool hcOK   = false;
bool dhtOK  = false;
bool ds18OK = false;

const uint8_t MAX_FALLOS = 3;

uint8_t fallosHC   = 0;
uint8_t fallosDHT  = 0;
uint8_t fallosDS   = 0;

/* Sensores que alimentan el IRH: HC-SR04 + DS18B20 + DHT11.
   El KY-018 no entra: es digital y no tiene forma de avisar
   que esta desconectado (siempre "parece" OK). La foto-
   resistencia tampoco entra al IRH: es informativa (aunque
   si participa en la probabilidad de evaporacion). */
uint8_t sensoresDatos = 0;


/* ============================================================
   CONFIGURACION TANQUE
   ============================================================ */

const float ALTURA_TANQUE = 100.0;


/* ============================================================
   TIEMPOS
   ============================================================ */

unsigned long ultimoDHT    = 0;
unsigned long ultimoDS     = 0;
unsigned long ultimoSerial = 0;
unsigned long ultimaOLED   = 0;

const unsigned long INTERVALO_DHT    = 2000;
const unsigned long INTERVALO_DS     = 1000;
const unsigned long INTERVALO_SERIAL = 2000;
const unsigned long INTERVALO_OLED   = 1000;


/* ============================================================
   ANALITICA / ESTADOS
   ============================================================ */

float IRH       = 0;   /* IRH final, ya con el ajuste de anomalias */
float IRHbase   = 0;   /* IRH antes de sumar el efecto de anomalias */
int   anomalias = 0;   /* contador de anomalias detectadas (ventana movil) */
int   cluster   = 0;   /* reservado: clasificacion K-Means OFFLINE (ver wiki) */

/* NUEVO en v6: probabilidad de evaporacion, 0-100%.
   NAN cuando no hay suficientes sensores para estimarla. */
float probabilidadEvaporacion = NAN;

enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_ALERTA,
  ESTADO_CRITICO,
  ESTADO_DEGRADADO,   /* faltan sensores: no se puede confirmar */
  ESTADO_ERROR        /* fallo total de sensores */
};

EstadoSistema estado = ESTADO_DEGRADADO;

const float UMBRAL_ALERTA         = 30.0;
const float UMBRAL_CRITICO        = 70.0;
const float UMBRAL_SALIDA_CRITICO = 60.0;   /* histeresis */

/* El IRH debe mantenerse alto este tiempo antes de declarar
   CRITICO. Evita que un pico de una sola lectura dispare la
   alarma. */
const unsigned long TIEMPO_CONFIRMACION_CRITICO = 5000;

unsigned long inicioCondicionCritica = 0;


/* ============================================================
   DETECCION DE ANOMALIAS (ventana movil sobre el IRH base)

   Idea: se guardan las ultimas N lecturas de IRHbase. Se
   calcula el promedio y la desviacion estandar de esa
   ventana (sin contar la lectura actual). Si la lectura
   actual se aleja mas de UMBRAL_DESVIACIONES desviaciones
   estandar del promedio reciente, se marca como anomalia.
   ============================================================ */

const uint8_t TAM_VENTANA_ANOMALIA = 10;

float    historialIRH[TAM_VENTANA_ANOMALIA];
uint8_t  indiceHistorial   = 0;
uint8_t  muestrasGuardadas = 0;   /* cuantas casillas ya tienen dato real */

const float UMBRAL_DESVIACIONES   = 2.0;   /* que tan lejos del promedio cuenta como anomalia */
const int   MAX_ANOMALIAS_CONTADAS = 4;    /* tope para que no se dispare el IRH sin control */

bool detectarAnomalia(float valorActual) {

  if (muestrasGuardadas < 3) {

    historialIRH[indiceHistorial] = valorActual;
    indiceHistorial = (indiceHistorial + 1) % TAM_VENTANA_ANOMALIA;
    muestrasGuardadas++;

    return false;
  }

  float suma = 0;
  for (uint8_t i = 0; i < muestrasGuardadas; i++) {
    suma += historialIRH[i];
  }
  float promedio = suma / muestrasGuardadas;

  float sumaCuadrados = 0;
  for (uint8_t i = 0; i < muestrasGuardadas; i++) {
    float diferencia = historialIRH[i] - promedio;
    sumaCuadrados += diferencia * diferencia;
  }
  float desviacion = sqrt(sumaCuadrados / muestrasGuardadas);

  const float DESVIACION_MINIMA = 2.0;
  if (desviacion < DESVIACION_MINIMA) {
    desviacion = DESVIACION_MINIMA;
  }

  bool esAnomalia = fabs(valorActual - promedio) > (UMBRAL_DESVIACIONES * desviacion);

  historialIRH[indiceHistorial] = valorActual;
  indiceHistorial = (indiceHistorial + 1) % TAM_VENTANA_ANOMALIA;
  if (muestrasGuardadas < TAM_VENTANA_ANOMALIA) {
    muestrasGuardadas++;
  }

  return esAnomalia;
}


/* ============================================================
   HELPERS DE ESTADO
   ============================================================ */

const __FlashStringHelper* textoEstado() {
  switch (estado) {
    case ESTADO_NORMAL:    return F("NORMAL");
    case ESTADO_ALERTA:    return F("ALERTA");
    case ESTADO_CRITICO:   return F("CRITICO");
    case ESTADO_DEGRADADO: return F("DEGRAD");
    default:               return F("ERROR");
  }
}

char letraEstado() {
  switch (estado) {
    case ESTADO_NORMAL:    return 'N';
    case ESTADO_ALERTA:    return 'A';
    case ESTADO_CRITICO:   return 'C';
    case ESTADO_DEGRADADO: return 'D';
    default:               return 'E';
  }
}

/* Datos confiables = tengo el nivel (el corazon del sistema)
   y al menos un sensor de temperatura/humedad para respaldar.
   Sin esto no se permite declarar CRITICO. */
bool datosConfiables() {
  return hcOK && (ds18OK || dhtOK);
}

/* NUEVO en v7: version en palabras de la probabilidad de
   evaporacion, para no obligar a nadie a interpretar un
   numero suelto en la pantalla. */
const __FlashStringHelper* textoEvaporacion() {
  if (isnan(probabilidadEvaporacion)) return F("--");
  if (probabilidadEvaporacion < 30)   return F("BAJA");
  if (probabilidadEvaporacion < 60)   return F("MEDIA");
  return F("ALTA");
}

/* NUEVO en v7: version en palabras de si hay anomalias
   activas ahora mismo (en vez de mostrar solo el contador). */
const __FlashStringHelper* textoAnomalia() {
  return (anomalias > 0) ? F("SI") : F("NO");
}


/* ============================================================
   HC-SR04
   ============================================================ */

float leerDistancia() {

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(PIN_TRIG, LOW);

  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 30000);

  if (duracion == 0) {
    return NAN;
  }

  float d = duracion / 58.2;

  if (d < 2 || d > 400) {
    return NAN;
  }

  return d;
}


/* ============================================================
   CALCULAR NIVEL
   ============================================================ */

void calcularNivel() {

  float d = leerDistancia();

  if (isnan(d)) {

    if (fallosHC < MAX_FALLOS) {
      fallosHC++;
    }

    if (fallosHC >= MAX_FALLOS) {
      hcOK      = false;
      distancia = NAN;
      nivel     = NAN;
    }

    return;
  }

  fallosHC  = 0;
  hcOK      = true;
  distancia = d;

  float alturaAgua = ALTURA_TANQUE - distancia;

  nivel = (alturaAgua / ALTURA_TANQUE) * 100.0;
  nivel = constrain(nivel, 0, 100);
}


/* ============================================================
   DHT11
   ============================================================ */

void leerDHT() {

  if (millis() - ultimoDHT < INTERVALO_DHT) return;

  ultimoDHT = millis();

  float nuevaHumedad     = dht.readHumidity();
  float nuevaTemperatura = dht.readTemperature();

  if (isnan(nuevaHumedad) || isnan(nuevaTemperatura)) {

    if (fallosDHT < MAX_FALLOS) {
      fallosDHT++;
    }

    if (fallosDHT >= MAX_FALLOS) {
      dhtOK               = false;
      humedad             = NAN;
      temperaturaAmbiente = NAN;
    }

    return;
  }

  fallosDHT           = 0;
  dhtOK               = true;
  humedad             = nuevaHumedad;
  temperaturaAmbiente = nuevaTemperatura;
}


/* ============================================================
   DS18B20
   ============================================================ */

void leerTemperaturaAgua() {

  if (millis() - ultimoDS < INTERVALO_DS) return;

  ultimoDS = millis();

  sensorAgua.requestTemperatures();

  float t = sensorAgua.getTempCByIndex(0);

  /* 85 C suele indicar conversion inicial fallida. */
  bool lecturaMala =
    (t == DEVICE_DISCONNECTED_C) ||
    (t < -50) ||
    (t > 125) ||
    (t == 85.0);

  if (lecturaMala) {

    if (fallosDS < MAX_FALLOS) {
      fallosDS++;
    }

    if (fallosDS >= MAX_FALLOS) {
      ds18OK          = false;
      temperaturaAgua = NAN;
    }

    return;
  }

  fallosDS        = 0;
  ds18OK          = true;
  temperaturaAgua = t;
}


/* ============================================================
   LUZ (KY-018 digital)
   ============================================================ */

void leerLuz() {
  luz = (digitalRead(PIN_LUZ) == LOW);
}


/* ============================================================
   FOTORESISTENCIA (analogica, nivel de luminosidad 0-100%)
   ============================================================ */

void leerFotoresistencia() {

  int lecturaCruda = analogRead(PIN_FOTORESISTENCIA);  /* 0-1023 */

  float porcentaje = (lecturaCruda / 1023.0) * 100.0;

  if (INVERTIR_FOTORESISTENCIA) {
    porcentaje = 100.0 - porcentaje;
  }

  luminosidad = constrain(porcentaje, 0, 100);
}


/* ============================================================
   IRH

   Se normaliza sobre los sensores DISPONIBLES. Aqui se calcula
   el IRH BASE (sin anomalias); el ajuste por anomalias se
   aplica despues en actualizarAnaliticaIRH().
   ============================================================ */

void calcularIRHBase() {

  float puntos    = 0;
  float puntosMax = 0;

  sensoresDatos = 0;

  /* NIVEL */
  if (hcOK && !isnan(nivel)) {

    sensoresDatos++;
    puntosMax += 50;

    if (nivel < 15 || nivel > 95)      puntos += 50;
    else if (nivel < 30 || nivel > 85) puntos += 25;
  }

  /* TEMPERATURA AGUA */
  if (ds18OK && !isnan(temperaturaAgua)) {

    sensoresDatos++;
    puntosMax += 30;

    if (temperaturaAgua >= 35)      puntos += 30;
    else if (temperaturaAgua >= 30) puntos += 15;
  }

  /* HUMEDAD */
  if (dhtOK && !isnan(humedad)) {

    sensoresDatos++;
    puntosMax += 15;

    if (humedad >= 80) puntos += 15;
  }

  if (puntosMax == 0) {
    IRHbase = 0;      /* sin datos: el IRH no significa nada */
    return;
  }

  IRHbase = (puntos / puntosMax) * 100.0;
  IRHbase = constrain(IRHbase, 0, 100);
}


void actualizarAnaliticaIRH() {

  if (sensoresDatos == 0) {
    IRH = 0;
    return;
  }

  bool anomaliaDetectada = detectarAnomalia(IRHbase);

  if (anomaliaDetectada && anomalias < MAX_ANOMALIAS_CONTADAS) {
    anomalias++;
  } else if (!anomaliaDetectada && anomalias > 0) {
    anomalias--;
  }

  IRH = IRHbase + (anomalias * 5.0);
  IRH = constrain(IRH, 0, 100);
}


/* ============================================================
   PROBABILIDAD DE EVAPORACION (heuristica, 0-100%)
   NUEVO en v6

   No es un modelo fisico exacto (para eso harian falta datos
   como viento y presion, que este hardware no mide). Es un
   indice compuesto que combina VARIAS senales -temperatura
   del agua, temperatura ambiente, humedad y luminosidad (como
   proxy de radiacion/calor solar)- para que ninguna variable
   sola pueda mover el numero por si misma.

   Se normaliza sobre los sensores DISPONIBLES, igual que el
   IRH: si falta un sensor, no resta puntaje, simplemente no
   participa ni el ni su peso.

   Pesos usados (suman 100 cuando todo esta disponible):
     - Temperatura agua      : 35
     - Humedad                : 30
     - Luminosidad             : 20
     - Temperatura ambiente   : 15
   ============================================================ */

void calcularProbabilidadEvaporacion() {

  float puntos    = 0;
  float puntosMax = 0;

  /* TEMPERATURA AGUA (peso 35): mapea 0-40 C -> 0-100% */
  if (ds18OK && !isnan(temperaturaAgua)) {
    float factor = (temperaturaAgua / 40.0) * 100.0;
    factor = constrain(factor, 0, 100);
    puntos    += factor * 0.35;
    puntosMax += 100.0 * 0.35;
  }

  /* HUMEDAD (peso 30): mas humedad -> MENOS evaporacion */
  if (dhtOK && !isnan(humedad)) {
    float factor = 100.0 - humedad;
    factor = constrain(factor, 0, 100);
    puntos    += factor * 0.30;
    puntosMax += 100.0 * 0.30;
  }

  /* LUMINOSIDAD (peso 20): proxy de calor/radiacion solar */
  if (!isnan(luminosidad)) {
    puntos    += luminosidad * 0.20;
    puntosMax += 100.0 * 0.20;
  }

  /* TEMPERATURA AMBIENTE (peso 15): mapea 0-40 C -> 0-100% */
  if (dhtOK && !isnan(temperaturaAmbiente)) {
    float factor = (temperaturaAmbiente / 40.0) * 100.0;
    factor = constrain(factor, 0, 100);
    puntos    += factor * 0.15;
    puntosMax += 100.0 * 0.15;
  }

  if (puntosMax == 0) {
    probabilidadEvaporacion = NAN;   /* sin datos suficientes */
    return;
  }

  probabilidadEvaporacion = (puntos / puntosMax) * 100.0;
  probabilidadEvaporacion = constrain(probabilidadEvaporacion, 0, 100);
}


/* ============================================================
   MAQUINA DE ESTADOS

   Reglas:
   - 0 sensores de datos      -> ERROR (pantalla + buzzer)
   - datos no confiables      -> DEGRADADO (nunca CRITICO)
   - CRITICO                  -> solo si el IRH se mantiene
                                 alto durante el tiempo de
                                 confirmacion
   - salir de CRITICO         -> con histeresis
   ============================================================ */

void actualizarEstado() {

  if (sensoresDatos == 0) {
    estado                 = ESTADO_ERROR;
    inicioCondicionCritica = 0;
    return;
  }

  if (!datosConfiables()) {
    estado                 = ESTADO_DEGRADADO;
    inicioCondicionCritica = 0;
    return;
  }

  if (IRH >= UMBRAL_CRITICO) {

    if (inicioCondicionCritica == 0) {
      inicioCondicionCritica = millis();
    }

  } else {

    if (!(estado == ESTADO_CRITICO && IRH >= UMBRAL_SALIDA_CRITICO)) {
      inicioCondicionCritica = 0;
    }
  }

  bool criticoConfirmado =
    (inicioCondicionCritica != 0) &&
    (millis() - inicioCondicionCritica >= TIEMPO_CONFIRMACION_CRITICO);

  if (estado == ESTADO_CRITICO && IRH >= UMBRAL_SALIDA_CRITICO) {
    criticoConfirmado = true;
  }

  if (criticoConfirmado) {
    estado = ESTADO_CRITICO;
  }
  else if (IRH >= UMBRAL_ALERTA || inicioCondicionCritica != 0) {
    estado = ESTADO_ALERTA;
  }
  else {
    estado = ESTADO_NORMAL;
  }
}


/* ============================================================
   SALIDAS: LEDS + BUZZER   (sin delay, todo con millis)
   ============================================================ */

void actualizarSalidas() {

  bool parpadeoRapido = ((millis() / 250) % 2) == 0;
  bool parpadeoLento  = ((millis() / 600) % 2) == 0;

  switch (estado) {

    case ESTADO_NORMAL:
      digitalWrite(PIN_LED_VERDE,    HIGH);
      digitalWrite(PIN_LED_AMARILLO, LOW);
      digitalWrite(PIN_LED_ROJO,     LOW);
      noTone(PIN_BUZZER);
      break;

    case ESTADO_ALERTA:
      digitalWrite(PIN_LED_VERDE,    LOW);
      digitalWrite(PIN_LED_AMARILLO, HIGH);
      digitalWrite(PIN_LED_ROJO,     LOW);
      noTone(PIN_BUZZER);
      break;

    case ESTADO_DEGRADADO:
      digitalWrite(PIN_LED_VERDE,    LOW);
      digitalWrite(PIN_LED_AMARILLO, parpadeoLento ? HIGH : LOW);
      digitalWrite(PIN_LED_ROJO,     LOW);
      noTone(PIN_BUZZER);
      break;

    case ESTADO_CRITICO:
      digitalWrite(PIN_LED_VERDE,    LOW);
      digitalWrite(PIN_LED_AMARILLO, LOW);
      digitalWrite(PIN_LED_ROJO,     HIGH);
      tone(PIN_BUZZER, 1500);
      break;

    case ESTADO_ERROR:
    default:
      digitalWrite(PIN_LED_VERDE,    LOW);
      digitalWrite(PIN_LED_AMARILLO, LOW);
      digitalWrite(PIN_LED_ROJO,     parpadeoRapido ? HIGH : LOW);

      if (parpadeoRapido) tone(PIN_BUZZER, 2500);
      else                noTone(PIN_BUZZER);
      break;
  }
}


/* ============================================================
   OLED - PANTALLA DE ERROR
   ============================================================ */

void pantallaError() {

  oled.clear();

  bool parpadeo = ((millis() / 500) % 2) == 0;

  oled.setCursor(1, 2);
  oled.print(parpadeo ? F("*** ERROR ***") : F("             "));

  oled.setCursor(1, 3);
  oled.print(F("SIN SENSORES"));

  oled.setCursor(1, 4);
  oled.print(F("HC-SR04 : FALLA"));

  oled.setCursor(1, 5);
  oled.print(F("DHT11   : FALLA"));

  oled.setCursor(1, 6);
  oled.print(F("DS18B20 : FALLA"));

  oled.setCursor(1, 7);
  oled.print(F("Revisar cableado"));
}


/* ============================================================
   OLED
   ============================================================ */

void actualizarOLED() {

  /* NO USAMOS FILAS 0 Y 1 POR EL DAÑO DE LA PANTALLA. */

  if (estado == ESTADO_ERROR) {
    pantallaError();
    return;
  }

  oled.clear();

  /* v7: pantalla reordenada para que se lea como frases
     cortas en vez de codigos abreviados. El IRH numerico
     crudo se saca de la vista principal (sigue disponible
     completo por Serial); aqui lo que importa es que se
     entienda de un vistazo. */

  /* FILA 2 - estado del sistema, en palabras */
  oled.setCursor(1, 2);
  oled.print(F("Estado: "));
  oled.print(textoEstado());

  /* FILA 3 - anomalias, en palabras (no un contador crudo) */
  oled.setCursor(1, 3);
  oled.print(F("Anomalia: "));
  oled.print(textoAnomalia());

  /* FILA 4 - nivel de tanque y temp. del agua */
  oled.setCursor(1, 4);
  oled.print(F("Niv:"));
  if (isnan(nivel)) oled.print(F("--"));
  else              oled.print(nivel, 0);
  oled.print(F("% Agua:"));
  if (isnan(temperaturaAgua)) oled.print(F("--"));
  else                        oled.print(temperaturaAgua, 0);
  oled.print(F("C"));

  /* FILA 5 - ambiente y humedad */
  oled.setCursor(1, 5);
  oled.print(F("Amb:"));
  if (isnan(temperaturaAmbiente)) oled.print(F("--"));
  else                            oled.print(temperaturaAmbiente, 0);
  oled.print(F("C Hum:"));
  if (isnan(humedad)) oled.print(F("--"));
  else                oled.print(humedad, 0);
  oled.print(F("%"));

  /* FILA 6 - probabilidad de evaporacion, numero + categoria
     en palabras (esto es lo que pidio mostrarse claro) */
  oled.setCursor(1, 6);
  oled.print(F("Evapora: "));
  if (isnan(probabilidadEvaporacion)) {
    oled.print(F("--"));
  } else {
    oled.print((int)probabilidadEvaporacion);
    oled.print(F("% "));
    oled.print(textoEvaporacion());
  }

  /* FILA 7 - confirmacion / sensores activos */
  oled.setCursor(1, 7);

  if (estado == ESTADO_DEGRADADO) {
    oled.print(F("Sin confirmar"));
  }
  else if (inicioCondicionCritica != 0 && estado != ESTADO_CRITICO) {
    oled.print(F("Verificando..."));
  }
  else {
    oled.print(F("Sensores: "));
    oled.print(sensoresDatos);
    oled.print(F("/3"));
  }
}


/* ============================================================
   SERIAL - ESTADO DE SENSORES
   ============================================================ */

void imprimirEstadoSensores() {

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("      WATER GUARD - SENSORES"));
  Serial.println(F("================================"));

  /* HC-SR04 */
  Serial.print(F("[HC-SR04]   "));
  if (hcOK) {
    Serial.print(F("OK - "));
    Serial.print(distancia, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.print(F("CAIDO (fallos: "));
    Serial.print(fallosHC);
    Serial.println(F(")"));
  }

  /* DHT11 */
  Serial.print(F("[DHT11]     "));
  if (dhtOK) {
    Serial.print(F("OK - Ambiente: "));
    Serial.print(temperaturaAmbiente, 1);
    Serial.print(F(" C / Humedad: "));
    Serial.print(humedad, 1);
    Serial.println(F(" %"));
  } else {
    Serial.print(F("CAIDO (fallos: "));
    Serial.print(fallosDHT);
    Serial.println(F(")"));
  }

  /* DS18B20 */
  Serial.print(F("[DS18B20]   "));
  if (ds18OK) {
    Serial.print(F("OK - Agua: "));
    Serial.print(temperaturaAgua, 1);
    Serial.println(F(" C"));
  } else {
    Serial.print(F("CAIDO (fallos: "));
    Serial.print(fallosDS);
    Serial.println(F(")"));
  }

  /* KY-018 */
  Serial.print(F("[KY-018]    "));
  Serial.print(F("Luz: "));
  Serial.print(luz ? F("SI") : F("NO"));
  Serial.println(F("  (digital, sin diagnostico)"));

  /* Fotoresistencia */
  Serial.print(F("[FOTOR.]    "));
  Serial.print(F("Luminosidad: "));
  if (isnan(luminosidad)) Serial.println(F("--"));
  else {
    Serial.print(luminosidad, 1);
    Serial.println(F(" %"));
  }

  Serial.println();
  Serial.print(F("Sensores de datos OK: "));
  Serial.print(sensoresDatos);
  Serial.println(F("/3"));

  if (sensoresDatos == 3)
    Serial.println(F(">>> TODOS LOS SENSORES OK <<<"));
  else if (sensoresDatos == 0)
    Serial.println(F(">>> ERROR TOTAL - REVISAR CONEXIONES <<<"));
  else
    Serial.println(F(">>> OPERANDO DEGRADADO <<<"));

  Serial.println(F("================================"));
}


/* ============================================================
   SERIAL - DATOS

   NOTA: al final imprime una linea tipo CSV (LOG,...) para
   poder pegar la salida del Monitor Serial en un archivo y
   procesarla despues en Python (analisis offline de K-Means
   que va en la wiki, y ahora tambien queda ahi el dato de
   probabilidad de evaporacion por si sirve para comparar
   contra el consumo real de agua).
   ============================================================ */

void imprimirDatosSerial() {

  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("         LECTURAS ACTUALES"));
  Serial.println(F("--------------------------------"));

  Serial.print(F("Nivel:              "));
  if (isnan(nivel)) Serial.println(F("ERROR"));
  else { Serial.print(nivel, 1); Serial.println(F(" %")); }

  Serial.print(F("Distancia:          "));
  if (isnan(distancia)) Serial.println(F("ERROR"));
  else { Serial.print(distancia, 1); Serial.println(F(" cm")); }

  Serial.print(F("Temperatura agua:   "));
  if (isnan(temperaturaAgua)) Serial.println(F("ERROR"));
  else { Serial.print(temperaturaAgua, 1); Serial.println(F(" C")); }

  Serial.print(F("Temperatura amb.:   "));
  if (isnan(temperaturaAmbiente)) Serial.println(F("ERROR"));
  else { Serial.print(temperaturaAmbiente, 1); Serial.println(F(" C")); }

  Serial.print(F("Humedad:            "));
  if (isnan(humedad)) Serial.println(F("ERROR"));
  else { Serial.print(humedad, 1); Serial.println(F(" %")); }

  Serial.print(F("Luz (KY-018):       "));
  Serial.println(luz ? F("SI") : F("NO"));

  Serial.print(F("Luminosidad:        "));
  if (isnan(luminosidad)) Serial.println(F("ERROR"));
  else { Serial.print(luminosidad, 1); Serial.println(F(" %")); }

  /* v7: la evaporacion es su propia lectura, al mismo nivel
     que nivel/temperatura/humedad, no un derivado escondido
     entre los datos de IRH. Incluye numero + categoria en
     palabras (BAJA/MEDIA/ALTA). */
  Serial.print(F("Posible evaporacion:"));
  if (isnan(probabilidadEvaporacion)) {
    Serial.println(F(" SIN DATOS"));
  } else {
    Serial.print(F(" "));
    Serial.print(probabilidadEvaporacion, 1);
    Serial.print(F(" % ("));
    Serial.print(textoEvaporacion());
    Serial.println(F(")"));
  }

  Serial.println();
  Serial.println(F("--- Analitica interna (IRH) ---"));

  Serial.print(F("IRH base:           "));
  if (sensoresDatos == 0) Serial.println(F("SIN DATOS"));
  else                    Serial.println(IRHbase, 1);

  Serial.print(F("IRH final:          "));
  if (sensoresDatos == 0) Serial.println(F("SIN DATOS"));
  else                    Serial.println(IRH, 1);

  Serial.print(F("Confiabilidad:      "));
  Serial.println(datosConfiables() ? F("OK") : F("INSUFICIENTE"));

  Serial.print(F("Anomalias activas:  "));
  Serial.print(anomalias);
  Serial.print(F(" ("));
  Serial.print(textoAnomalia());
  Serial.println(F(")"));

  Serial.print(F("Estado:             "));
  Serial.println(textoEstado());

  if (inicioCondicionCritica != 0 && estado != ESTADO_CRITICO) {
    Serial.print(F("  -> condicion critica en verificacion ("));
    Serial.print((millis() - inicioCondicionCritica) / 1000);
    Serial.print(F("s / "));
    Serial.print(TIEMPO_CONFIRMACION_CRITICO / 1000);
    Serial.println(F("s)"));
  }

  Serial.println(F("--------------------------------"));

  /* Linea CSV para copiar/pegar y analizar offline.
     v6: se agrego la columna de probabilidad de evaporacion
     al final. */
  Serial.print(F("LOG,"));
  Serial.print(millis());       Serial.print(F(","));
  Serial.print(nivel);          Serial.print(F(","));
  Serial.print(temperaturaAgua);Serial.print(F(","));
  Serial.print(temperaturaAmbiente); Serial.print(F(","));
  Serial.print(humedad);        Serial.print(F(","));
  Serial.print(luminosidad);    Serial.print(F(","));
  Serial.print(IRH);            Serial.print(F(","));
  Serial.print(anomalias);      Serial.print(F(","));
  Serial.println(probabilidadEvaporacion);
}


/* ============================================================
   SETUP
   ============================================================ */

void setup() {

  Serial.begin(9600);
  delay(500);

  pinMode(PIN_TRIG,         OUTPUT);
  pinMode(PIN_ECHO,         INPUT);
  pinMode(PIN_LED_VERDE,    OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO,     OUTPUT);
  pinMode(PIN_BUZZER,       OUTPUT);
  pinMode(PIN_LUZ,          INPUT);
  /* PIN_FOTORESISTENCIA (A0) no necesita pinMode */

  /* ---------- OLED ---------- */
  oled.begin();
  oled.setFont(u8x8_font_amstrad_cpc_extended_r);
  oled.clear();

  oled.setCursor(1, 2);
  oled.print(F("WATER GUARD"));
  oled.setCursor(1, 4);
  oled.print(F("Iniciando..."));

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("      WATER GUARD INICIANDO"));
  Serial.println(F("================================"));

  /* ---------- DHT11 ---------- */
  Serial.println();
  Serial.println(F("[DHT11] Iniciando..."));

  dht.begin();
  delay(1500);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    dhtOK               = true;
    fallosDHT           = 0;
    humedad             = h;
    temperaturaAmbiente = t;

    Serial.print(F("[DHT11] OK -> Ambiente: "));
    Serial.print(t, 1);
    Serial.print(F(" C | Humedad: "));
    Serial.print(h, 1);
    Serial.println(F(" %"));
  } else {
    dhtOK     = false;
    fallosDHT = MAX_FALLOS;
    Serial.println(F("[DHT11] ERROR - no responde"));
  }

  /* ---------- DS18B20 ---------- */
  Serial.println();
  Serial.println(F("[DS18B20] Buscando sensor..."));

  sensorAgua.begin();

  uint8_t cantidad = sensorAgua.getDeviceCount();

  Serial.print(F("[DS18B20] Dispositivos encontrados: "));
  Serial.println(cantidad);

  if (cantidad > 0) {

    sensorAgua.setResolution(12);
    sensorAgua.requestTemperatures();

    float temp = sensorAgua.getTempCByIndex(0);

    Serial.print(F("[DS18B20] Lectura: "));
    Serial.println(temp);

    if (temp != DEVICE_DISCONNECTED_C &&
        temp >= -50 && temp <= 125 && temp != 85.0) {

      ds18OK          = true;
      fallosDS        = 0;
      temperaturaAgua = temp;

      Serial.print(F("[DS18B20] >>> OK - AGUA: "));
      Serial.print(temp, 2);
      Serial.println(F(" C <<<"));

    } else {
      ds18OK   = false;
      fallosDS = MAX_FALLOS;
      Serial.println(F("[DS18B20] DETECTADO PERO NO PUDO LEER"));
    }

  } else {
    ds18OK   = false;
    fallosDS = MAX_FALLOS;
    Serial.println(F("[DS18B20] >>> NO DETECTADO <<<"));
    Serial.println(F("Revisa DATA -> D4"));
    Serial.println(F("Revisa resistencia 4.7K entre DATA y 5V"));
  }

  /* ---------- HC-SR04 ---------- */
  Serial.println();
  Serial.println(F("[HC-SR04] Probando..."));

  fallosHC = MAX_FALLOS - 1;   /* un fallo mas y queda caido */
  calcularNivel();

  if (hcOK) {
    Serial.print(F("[HC-SR04] OK -> "));
    Serial.print(distancia, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("[HC-SR04] ERROR / SIN ECO"));
  }

  /* ---------- KY-018 ---------- */
  Serial.println();
  Serial.println(F("[KY-018] Probando..."));

  leerLuz();

  Serial.print(F("[KY-018] Lectura: "));
  Serial.println(luz ? F("LUZ") : F("OSCURO"));

  /* ---------- FOTORESISTENCIA ---------- */
  Serial.println();
  Serial.println(F("[FOTOR.] Probando..."));

  leerFotoresistencia();

  Serial.print(F("[FOTOR.] Luminosidad: "));
  Serial.print(luminosidad, 1);
  Serial.println(F(" %"));

  /* ---------- PRIMER ESTADO ---------- */
  calcularIRHBase();
  actualizarAnaliticaIRH();
  calcularProbabilidadEvaporacion();
  actualizarEstado();
  actualizarSalidas();
  actualizarOLED();

  Serial.println();
  Serial.println(F("================================"));
  Serial.print(F("SENSORES DE DATOS OK: "));
  Serial.print(sensoresDatos);
  Serial.println(F("/3"));

  if (sensoresDatos == 0)
    Serial.println(F(">>> ERROR TOTAL: NINGUN SENSOR RESPONDE <<<"));
  else if (sensoresDatos < 3)
    Serial.println(F(">>> ARRANQUE DEGRADADO <<<"));
  else
    Serial.println(F(">>> TODOS LOS SENSORES FUNCIONANDO <<<"));

  Serial.println(F("================================"));
  Serial.println(F("Sistema listo."));
  Serial.println(F("Monitor Serial: 9600 baudios"));
}


/* ============================================================
   LOOP
   ============================================================ */

void loop() {

  unsigned long ahora = millis();

  /* ---------- SENSORES ---------- */
  calcularNivel();
  leerDHT();
  leerTemperaturaAgua();
  leerLuz();
  leerFotoresistencia();

  /* ---------- ANALITICA + ESTADO ---------- */
  calcularIRHBase();
  actualizarAnaliticaIRH();
  calcularProbabilidadEvaporacion();
  actualizarEstado();

  /* ---------- SALIDAS ---------- */
  actualizarSalidas();

  /* ---------- OLED ---------- */
  if (ahora - ultimaOLED >= INTERVALO_OLED) {
    ultimaOLED = ahora;
    actualizarOLED();
  }

  /* ---------- SERIAL ---------- */
  if (ahora - ultimoSerial >= INTERVALO_SERIAL) {
    ultimoSerial = ahora;
    imprimirEstadoSensores();
    imprimirDatosSerial();
  }

  delay(50);
}
