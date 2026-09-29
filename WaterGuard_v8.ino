/* ============================================================
   WATER GUARD - v8 (ESP32)          Challenge #2 - IoT 2026-2
   ============================================================

   Evolucion de la v7 (Arduino Uno R3, solo alertas locales) a
   ESP32 DevKit V1 (WROOM-32, CP2102, 30 pines) con tablero de
   control web embebido en la WLAN de la Alcaldia.

   QUE CAMBIA FRENTE A v7
   - Placa: ESP32. ADC de 12 bits, logica de 3.3 V.
   - Medicion fuera del hilo principal (requisito del reto):
       * ISR del pin ECHO (HC-SR04): mide el ancho del pulso con
         esp_timer y despierta a la tarea de sensores.
       * ISR del boton fisico: silencia la alarma in situ.
       * Tareas FreeRTOS: tSensores, tAnalitica, tSalidas.
         El loop() solo atiende el servidor web.
   - Tablero de control embebido (HTTP, SIN MQTT):
       valores actuales, historico reciente (15 min), eventos
       y notificaciones, y boton para silenciar la alarma.
   - Acceso restringido en 3 capas:
       1) WPA2 de la WLAN (lo pone la Alcaldia).
       2) Filtro de subred: solo IPs de la misma red local.
          (Opcional: lista blanca de IPs con reserva DHCP.)
       3) Login con usuario/clave (clave guardada como hash
          SHA-256) -> cookie de sesion HttpOnly atada a la IP,
          con expiracion y bloqueo tras intentos fallidos.
   - Nuevas variables: presion atmosferica (BMP280) y
     deficit de presion de vapor (ley de Dalton), que ahora
     alimentan el indice de evaporacion potencial.
   - Nueva regla de fusion explicita del enunciado:
     "descenso anomalo de nivel bajo calor extremo".
   - Nivel REDUNDANTE: HC-SR04 (ultrasonido, desde arriba) +
     MD-PS002/HX710B (presion hidrostatica, manguera al fondo).
     Si coinciden se promedian; si discrepan se usa el MENOR
     (criterio conservador para sequia) y se genera un evento.
     Si uno falla, el otro mantiene el sistema operando.

   MAPA DE PINES (ESP32 DevKit V1 - 30 pines)
     OLED SSD1306 + BMP280 (I2C)  SDA 21, SCL 22
     HC-SR04  TRIG 25 | ECHO 26 (DIVISOR 1k/2k: ECHO sale a 5 V)
     DHT11    DATA 4
     DS18B20  DATA 27 (pull-up 4.7k a 3.3 V)
     Fotoresistencia (divisor)  34 (ADC1, solo entrada)
     MD-PS002 + HX710B (nivel por presion)  OUT 32 | SCK 33
     (KY-018 retirado: "hay luz" se deriva de la fotoresistencia)
     LED verde 16 | LED amarillo 17 | LED rojo 19
     Buzzer 23 (via transistor NPN, alimentado desde VIN)
     Boton silenciar 18 (a GND, INPUT_PULLUP)
   Pines evitados: 0, 2, 12, 15 (arranque) y ADC2 (no sirve
   con WiFi activo).

   LIBRERIAS (Gestor de librerias del Arduino IDE)
     U8g2, DHT sensor library (Adafruit), Adafruit Unified
     Sensor, Adafruit BMP280, OneWire, DallasTemperature.
   Placa: "ESP32 Dev Module". Monitor Serial: 115200 baudios.
   ============================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_system.h"
#include "soc/gpio_reg.h"
#include "mbedtls/sha256.h"
#include "mbedtls/version.h"


/* ============================================================
   CONFIGURACION DE RED Y ACCESO  (EDITAR ANTES DE CARGAR)
   ============================================================ */

const char* WIFI_SSID = "WLAN_ALCALDIA";
const char* WIFI_PASS = "clave-de-la-red";
const char* NOMBRE_MDNS = "waterguard";     /* http://waterguard.local */

const char* USUARIO_TABLERO = "autoridad";

/* SHA-256 (hex, minusculas) de la clave del tablero.
   Valor por defecto = SHA-256("Sabana2026").
   Para cambiarla:  python -c "import hashlib;print(hashlib.sha256(b'NUEVA').hexdigest())" */
const char* HASH_CLAVE_TABLERO =
  "b7c965dc56b99de99ddd1c5df3436275f7b812183e1b35bf9917f971593a3185";

/* Lista blanca opcional (requiere reservas DHCP en el router). */
const bool USAR_LISTA_BLANCA = false;
const IPAddress LISTA_BLANCA[] = {
  IPAddress(192, 168, 1, 50),
  IPAddress(192, 168, 1, 51)
};
const uint8_t N_LISTA_BLANCA = sizeof(LISTA_BLANCA) / sizeof(LISTA_BLANCA[0]);


/* ============================================================
   PINES
   ============================================================ */

#define PIN_SDA             21
#define PIN_SCL             22
#define PIN_TRIG            25
#define PIN_ECHO            26
#define PIN_DHT             4
#define PIN_DS18B20         27
#define PIN_FOTORESISTENCIA 34
#define PIN_HX_OUT          32
#define PIN_HX_SCK          33
#define PIN_LED_VERDE       16
#define PIN_LED_AMARILLO    17
#define PIN_LED_ROJO        19
#define PIN_BUZZER          23
#define PIN_BOTON           18


/* ============================================================
   PARAMETROS DEL SISTEMA
   ============================================================ */

const float ALTURA_TANQUE = 100.0;     /* cm, distancia sensor-fondo */

/* MD-PS002 + HX710B: presion hidrostatica -> columna de agua.
   La punta de la manguera va al FONDO del tanque.
   HX_AUTO_TARA: al encender, la lectura actual se toma como
   "0 cm". Por eso la manguera debe estar FUERA del agua (o el
   tanque vacio) durante los primeros 2 s tras el arranque.
   HX_CUENTAS_POR_KPA: sensibilidad (depende del modulo).
   Calibracion en 2 puntos: anote "HX crudo" del Serial con la
   manguera fuera y luego sumergida H cm; entonces
     cuentas/kPa = (crudo_sumergido - crudo_aire) / (H * 0.0980665) */
const bool  HX_AUTO_TARA        = true;
long        HX_OFFSET           = 0;          /* se llena con la tara */
const float HX_CUENTAS_POR_KPA  = 325000.0;
const float TOLERANCIA_NIVEL    = 10.0;       /* % de diferencia HC vs presion */

/* Umbrales del IRH (Indice de Riesgo Hidrico, 0-100) */
const float UMBRAL_ALERTA         = 30.0;
const float UMBRAL_CRITICO        = 70.0;
const float UMBRAL_SALIDA_CRITICO = 60.0;   /* histeresis */
const unsigned long TIEMPO_CONFIRMACION_CRITICO = 5000;

/* Tasa de descenso de nivel (%/min). Estan escalados para el
   prototipo (tanque pequeno, demo de minutos). En campo se
   ajustan a la dinamica real del reservorio. */
const float UMBRAL_DESCENSO       = 1.0;
const float UMBRAL_DESCENSO_CRIT  = 3.0;

/* Condicion de "calor" para las reglas de fusion */
const float T_AMB_CALOR     = 26.0;   /* C */
const float T_AGUA_ALTA     = 28.0;   /* C */
const float T_AGUA_MEDIA    = 24.0;   /* C */
const float HUMEDAD_BAJA    = 35.0;   /* % */
const float HUMEDAD_MEDIA   = 50.0;   /* % */
const float EVAP_ALTA       = 60.0;   /* indice 0-100 */
const float EVAP_MEDIA      = 30.0;

/* Buzzer conectado DIRECTO al pin (sin transistor).
   false = buzzer ACTIVO (suena solo con voltaje)
   true  = buzzer PASIVO (necesita una frecuencia; se usa tone) */
const bool BUZZER_PASIVO = false;
const uint16_t FRECUENCIA_BUZZER = 2000;   /* Hz, solo si es pasivo */

/* Si la fotoresistencia da mas lectura con menos luz, poner true */
const bool INVERTIR_FOTORESISTENCIA = false;

/* Luminosidad (%) a partir de la cual se considera que "hay luz" */
const float UMBRAL_LUZ = 40.0;

/* Silencio de alarma: se rearma sola al salir de CRITICO/ERROR
   o despues de este tiempo, lo que ocurra primero. */
const uint32_t SILENCIO_MAX_MS = 10UL * 60UL * 1000UL;

/* Periodos de las tareas */
const uint32_t PERIODO_SENSORES_MS  = 500;
const uint32_t PERIODO_ANALITICA_MS = 1000;
const uint32_t PERIODO_SALIDAS_MS   = 50;
const uint32_t INTERVALO_DHT_MS     = 2000;
const uint32_t INTERVALO_DS_MS      = 1000;
const uint32_t INTERVALO_BMP_MS     = 1000;
const uint32_t INTERVALO_OLED_MS    = 1000;
const uint32_t INTERVALO_PAGINA_MS  = 4000;
const uint32_t INTERVALO_SERIAL_MS  = 2000;

const uint8_t MAX_FALLOS = 3;


/* ============================================================
   OBJETOS DE HARDWARE
   ============================================================ */

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
DHT dht(PIN_DHT, DHT11);
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorAgua(&oneWire);
Adafruit_BMP280 bmp;
WebServer server(80);
bool bmpPresente = false;   /* se detecta en setup() */
bool hxPresente  = false;   /* se detecta en setup() */


/* ============================================================
   ESTADO COMPARTIDO ENTRE TAREAS
   Todo acceso pasa por mtxDatos. El bus I2C (OLED + BMP280)
   se usa desde dos tareas distintas, asi que tiene su propio
   mutex mtxI2C.
   ============================================================ */

SemaphoreHandle_t mtxDatos;
SemaphoreHandle_t mtxI2C;

TaskHandle_t hSensores  = NULL;
TaskHandle_t hAnalitica = NULL;
TaskHandle_t hSalidas   = NULL;

enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_ALERTA,
  ESTADO_CRITICO,
  ESTADO_DEGRADADO,
  ESTADO_ERROR
};

struct Lecturas {
  float nivel       = NAN;   /* % FUSIONADO (el que usa la analitica) */
  float nivelUS     = NAN;   /* % por ultrasonido (HC-SR04) */
  float nivelP      = NAN;   /* % por presion (MD-PS002) */
  float colAgua     = NAN;   /* cm de columna de agua (MD-PS002) */
  long  hxCrudo     = 0;     /* lectura cruda del HX710B (calibracion) */
  float distancia   = NAN;   /* cm */
  float tAgua       = NAN;   /* C  */
  float tAmb        = NAN;   /* C  */
  float humedad     = NAN;   /* %  */
  float presion     = NAN;   /* hPa */
  float luminosidad = NAN;   /* %  */
  bool  luz   = false;
  bool  hcOK  = false;
  bool  hxOK  = false;
  bool  nivelOK = false;       /* hay al menos una fuente de nivel */
  bool  discrepanciaNivel = false;
  bool  dhtOK = false;
  bool  dsOK  = false;
  bool  bmpOK = false;
};

struct Analitica {
  float irh      = 0;
  float irhBase  = 0;
  float evap     = NAN;   /* indice de evaporacion potencial 0-100 */
  float dpv      = NAN;   /* deficit de presion de vapor, kPa */
  float descenso = NAN;   /* %/min, positivo = el nivel baja */
  int   anomalias = 0;
  uint8_t sensoresDatos = 0;
  bool  reglaDescensoCalor = false;
  bool  reglaNivelEvap     = false;
  bool  verificandoCritico = false;
  EstadoSistema estado = ESTADO_DEGRADADO;
};

Lecturas  lect;
Analitica ana;

bool     alarmaSilenciada = false;
uint32_t silencioDesdeMs  = 0;

/* --- Historico para el tablero: 1 muestra cada 5 s, 15 min --- */
const uint8_t  N_HIST = 180;
const uint32_t PASO_HIST_MS = 5000;

struct Muestra {
  uint32_t t;        /* segundos desde el arranque */
  float nivel, tAgua, tAmb, hum, pres, lum, evap, irh, colAgua;
};

Muestra  hist[N_HIST];
uint16_t histInicio = 0;
uint16_t histCount  = 0;

/* --- Registro de eventos (notificaciones del tablero) --- */
enum TipoEvento { EV_INFO = 0, EV_ALERTA = 1, EV_CRITICO = 2, EV_SEGURIDAD = 3 };

struct Evento {
  uint32_t id;
  uint32_t t;
  uint8_t  tipo;
  char     msg[72];
};

const uint8_t N_EVENTOS = 25;
Evento   eventos[N_EVENTOS];
uint8_t  evInicio = 0;
uint8_t  evCount  = 0;
uint32_t evSiguienteId = 1;

/* --- Seguridad del tablero: sesiones y bloqueo por intentos --- */
const uint8_t  N_SESIONES = 6;
const uint32_t DURACION_SESION_MS = 30UL * 60UL * 1000UL;   /* 30 min, se renueva con uso */

struct Sesion { char token[33]; uint32_t ip; uint32_t ultimoUso; bool activa; };
Sesion sesiones[N_SESIONES];

const uint8_t  N_BLOQUEOS = 8;
const uint8_t  MAX_INTENTOS = 5;
const uint32_t TIEMPO_BLOQUEO_MS = 2UL * 60UL * 1000UL;

struct Intentos { uint32_t ip; uint8_t fallos; uint32_t desde; };
Intentos intentos[N_BLOQUEOS];

/* Debe llamarse CON mtxDatos tomado */
void registrarEventoSinLock(uint8_t tipo, const char* texto) {
  uint8_t pos = (evInicio + evCount) % N_EVENTOS;
  if (evCount == N_EVENTOS) {
    evInicio = (evInicio + 1) % N_EVENTOS;
  } else {
    evCount++;
  }
  eventos[pos].id   = evSiguienteId++;
  eventos[pos].t    = millis() / 1000;
  eventos[pos].tipo = tipo;
  strncpy(eventos[pos].msg, texto, sizeof(eventos[pos].msg) - 1);
  eventos[pos].msg[sizeof(eventos[pos].msg) - 1] = '\0';
  Serial.printf("[EVENTO %lu] %s\n", (unsigned long)eventos[pos].id, eventos[pos].msg);
}

void registrarEvento(uint8_t tipo, const char* texto) {
  xSemaphoreTake(mtxDatos, portMAX_DELAY);
  registrarEventoSinLock(tipo, texto);
  xSemaphoreGive(mtxDatos);
}


/* ============================================================
   ISR 1: ECHO DEL HC-SR04
   Flanco de subida -> guarda el instante. Flanco de bajada ->
   calcula el ancho del pulso y despierta a tSensores con una
   notificacion de tarea. Se lee el pin por registro (seguro
   en IRAM) en lugar de digitalRead().
   ============================================================ */

volatile int64_t echoInicioUs  = 0;
volatile int64_t echoDuracionUs = 0;

void IRAM_ATTR isrEcho() {
  int64_t ahora = esp_timer_get_time();
  bool alto = (REG_READ(GPIO_IN_REG) >> PIN_ECHO) & 0x1;

  if (alto) {
    echoInicioUs = ahora;
  } else if (echoInicioUs != 0) {
    echoDuracionUs = ahora - echoInicioUs;
    echoInicioUs = 0;
    BaseType_t despertar = pdFALSE;
    if (hSensores != NULL) {
      vTaskNotifyGiveFromISR(hSensores, &despertar);
    }
    if (despertar) portYIELD_FROM_ISR();
  }
}


/* ============================================================
   ISR 2: BOTON FISICO DE SILENCIO
   Antirrebote por tiempo. Solo levanta una bandera; la logica
   la aplica tAnalitica.
   ============================================================ */

volatile bool    botonPresionado = false;
volatile int64_t ultimoBotonUs   = 0;

void IRAM_ATTR isrBoton() {
  int64_t ahora = esp_timer_get_time();
  if (ahora - ultimoBotonUs > 250000) {
    ultimoBotonUs = ahora;
    botonPresionado = true;
  }
}


/* ============================================================
   HELPERS DE TEXTO
   ============================================================ */

const char* textoEstado(EstadoSistema e) {
  switch (e) {
    case ESTADO_NORMAL:    return "NORMAL";
    case ESTADO_ALERTA:    return "ALERTA";
    case ESTADO_CRITICO:   return "CRITICO";
    case ESTADO_DEGRADADO: return "DEGRADADO";
    default:               return "ERROR";
  }
}

const char* textoEvaporacion(float evap) {
  if (isnan(evap))       return "--";
  if (evap < EVAP_MEDIA) return "BAJA";
  if (evap < EVAP_ALTA)  return "MEDIA";
  return "ALTA";
}

bool alarmaSonando(EstadoSistema e) {
  return (e == ESTADO_CRITICO || e == ESTADO_ERROR) && !alarmaSilenciada;
}


/* ============================================================
   TAREA DE SENSORES  (nucleo 1, prioridad 3)
   ============================================================ */

/* Un disparo del HC-SR04: la duracion la mide la ISR. */
float medirDistanciaUnaVez() {
  ulTaskNotifyTake(pdTRUE, 0);          /* limpiar notificaciones viejas */
  echoDuracionUs = 0;
  echoInicioUs   = 0;

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  /* Espera la notificacion de la ISR (max 40 ms ~ 6.8 m) */
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40)) == 0) {
    return NAN;
  }

  float d = echoDuracionUs / 58.2f;
  if (d < 2 || d > 400) return NAN;
  return d;
}

/* Mediana de 3 disparos: filtra ecos sueltos. */
float medirDistancia() {
  float v[3];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    float d = medirDistanciaUnaVez();
    if (!isnan(d)) v[n++] = d;
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  if (n == 0) return NAN;
  if (n == 1) return v[0];
  if (n == 2) return (v[0] + v[1]) / 2.0f;
  float a = v[0], b = v[1], c = v[2];
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

/* HX710B: protocolo serie de 2 hilos. Cuando OUT baja, hay dato
   listo; se leen 24 bits (MSB primero) y un pulso extra
   (25 en total = entrada diferencial a 10 SPS). SCK no puede
   quedar en alto mas de ~60 us, por eso los pulsos van en una
   seccion critica corta (~50 us). */
portMUX_TYPE muxHX = portMUX_INITIALIZER_UNLOCKED;

bool leerHX710B(long& crudo, uint32_t esperaMaxMs) {
  uint32_t t = millis();
  while (digitalRead(PIN_HX_OUT) == HIGH) {
    if (millis() - t > esperaMaxMs) return false;   /* no hay dato / desconectado */
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  uint32_t v = 0;
  portENTER_CRITICAL(&muxHX);
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_HX_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_HX_OUT) ? 1 : 0);
    digitalWrite(PIN_HX_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH);   /* pulso 25 */
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);
  portEXIT_CRITICAL(&muxHX);

  if (v == 0x7FFFFF || v == 0x800000) return false;   /* saturado */
  if (v & 0x800000) v |= 0xFF000000;                  /* complemento a 2 */
  crudo = (long)(int32_t)v;
  return true;
}

void tareaSensores(void* param) {

  uint8_t fallosHC = 0, fallosDHT = 0, fallosDS = 0, fallosBMP = 0, fallosHX = 0;
  uint32_t ultimoDHT = 0, ultimoDS = 0, ultimoBMP = 0;
  bool dsPedido = false;

  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    uint32_t ahora = millis();

    /* ---------- HC-SR04 (cada ciclo) ---------- */
    float d = medirDistancia();
    float nuevoNivel = NAN;
    if (isnan(d)) {
      if (fallosHC < MAX_FALLOS) fallosHC++;
    } else {
      fallosHC = 0;
      nuevoNivel = constrain((ALTURA_TANQUE - d) / ALTURA_TANQUE * 100.0f, 0, 100);
    }

    /* ---------- MD-PS002 / HX710B (nivel por presion) ---------- */
    long  crudoHX = 0;
    float col = NAN, nivelP = NAN;
    if (hxPresente) {
      if (leerHX710B(crudoHX, 150)) {
        float kPa = (crudoHX - HX_OFFSET) / HX_CUENTAS_POR_KPA;
        col = kPa * 1000.0f / 98.0665f;            /* Pa -> cm de agua */
        if (col < -5 || col > ALTURA_TANQUE * 1.5f) { col = NAN; if (fallosHX < MAX_FALLOS) fallosHX++; }
        else {
          fallosHX = 0;
          if (col < 0) col = 0;
          nivelP = constrain(col / ALTURA_TANQUE * 100.0f, 0, 100);
        }
      } else if (fallosHX < MAX_FALLOS) fallosHX++;
    }

    /* ---------- DHT11 ---------- */
    float h = NAN, t = NAN;
    bool dhtLeido = false;
    if (ahora - ultimoDHT >= INTERVALO_DHT_MS) {
      ultimoDHT = ahora;
      h = dht.readHumidity();
      t = dht.readTemperature();
      dhtLeido = true;
      if (isnan(h) || isnan(t)) { if (fallosDHT < MAX_FALLOS) fallosDHT++; }
      else fallosDHT = 0;
    }

    /* ---------- DS18B20 (conversion sin bloqueo) ---------- */
    float ta = NAN;
    bool dsLeido = false;
    if (ahora - ultimoDS >= INTERVALO_DS_MS) {
      ultimoDS = ahora;
      if (dsPedido) {
        ta = sensorAgua.getTempCByIndex(0);
        dsLeido = true;
        bool mala = (ta == DEVICE_DISCONNECTED_C) || ta < -50 || ta > 125 || ta == 85.0f;
        if (mala) { ta = NAN; if (fallosDS < MAX_FALLOS) fallosDS++; }
        else fallosDS = 0;
      }
      sensorAgua.requestTemperatures();   /* no bloquea: setWaitForConversion(false) */
      dsPedido = true;
    }

    /* ---------- BMP280 ---------- */
    float p = NAN;
    bool bmpLeido = false;
    if (ahora - ultimoBMP >= INTERVALO_BMP_MS) {
      ultimoBMP = ahora;
      bmpLeido = true;
      if (bmpPresente) {
        xSemaphoreTake(mtxI2C, portMAX_DELAY);
        p = bmp.readPressure() / 100.0f;
        xSemaphoreGive(mtxI2C);
      }
      if (isnan(p) || p < 500 || p > 1100) { p = NAN; if (fallosBMP < MAX_FALLOS) fallosBMP++; }
      else fallosBMP = 0;
    }

    /* ---------- Luz (digital) y fotoresistencia (ADC1) ---------- */
    float lum = analogRead(PIN_FOTORESISTENCIA) / 4095.0f * 100.0f;
    if (INVERTIR_FOTORESISTENCIA) lum = 100.0f - lum;
    lum = constrain(lum, 0, 100);
    bool hayLuz = (lum >= UMBRAL_LUZ);   /* reemplaza al KY-018 */

    /* ---------- Publicar al estado compartido ---------- */
    xSemaphoreTake(mtxDatos, portMAX_DELAY);

    if (!isnan(nuevoNivel)) {
      lect.hcOK = true; lect.distancia = d; lect.nivelUS = nuevoNivel;
    } else if (fallosHC >= MAX_FALLOS) {
      lect.hcOK = false; lect.distancia = NAN; lect.nivelUS = NAN;
    }

    lect.hxCrudo = crudoHX;
    if (!isnan(nivelP)) {
      lect.hxOK = true; lect.colAgua = col; lect.nivelP = nivelP;
    } else if (!hxPresente || fallosHX >= MAX_FALLOS) {
      lect.hxOK = false; lect.colAgua = NAN; lect.nivelP = NAN;
    }

    /* Fusion de las dos fuentes de nivel */
    lect.discrepanciaNivel = false;
    if (lect.hcOK && lect.hxOK) {
      if (fabsf(lect.nivelUS - lect.nivelP) <= TOLERANCIA_NIVEL) {
        lect.nivel = (lect.nivelUS + lect.nivelP) / 2.0f;
      } else {
        lect.nivel = min(lect.nivelUS, lect.nivelP);   /* conservador */
        lect.discrepanciaNivel = true;
      }
    } else if (lect.hcOK) {
      lect.nivel = lect.nivelUS;
    } else if (lect.hxOK) {
      lect.nivel = lect.nivelP;
    } else {
      lect.nivel = NAN;
    }
    lect.nivelOK = lect.hcOK || lect.hxOK;

    if (dhtLeido) {
      if (fallosDHT == 0) { lect.dhtOK = true; lect.humedad = h; lect.tAmb = t; }
      else if (fallosDHT >= MAX_FALLOS) { lect.dhtOK = false; lect.humedad = NAN; lect.tAmb = NAN; }
    }

    if (dsLeido) {
      if (fallosDS == 0) { lect.dsOK = true; lect.tAgua = ta; }
      else if (fallosDS >= MAX_FALLOS) { lect.dsOK = false; lect.tAgua = NAN; }
    }

    if (bmpLeido) {
      if (fallosBMP == 0) { lect.bmpOK = true; lect.presion = p; }
      else if (fallosBMP >= MAX_FALLOS) { lect.bmpOK = false; lect.presion = NAN; }
    }

    lect.luz = hayLuz;
    lect.luminosidad = lum;

    xSemaphoreGive(mtxDatos);

    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_SENSORES_MS));
  }
}


/* ============================================================
   ANALITICA
   ============================================================ */

/* --- Deteccion de anomalias sobre el IRH base (igual que v7):
       ventana movil, media y desviacion estandar. --- */
const uint8_t TAM_VENTANA_ANOMALIA = 10;
const float   UMBRAL_DESVIACIONES  = 2.0;
const int     MAX_ANOMALIAS        = 4;

float   ventanaIRH[TAM_VENTANA_ANOMALIA];
uint8_t idxVentana = 0;
uint8_t nVentana   = 0;

bool detectarAnomalia(float x) {
  bool anomala = false;
  if (nVentana >= 3) {
    float suma = 0;
    for (uint8_t i = 0; i < nVentana; i++) suma += ventanaIRH[i];
    float media = suma / nVentana;
    float sc = 0;
    for (uint8_t i = 0; i < nVentana; i++) { float df = ventanaIRH[i] - media; sc += df * df; }
    float desv = sqrtf(sc / nVentana);
    if (desv < 2.0f) desv = 2.0f;
    anomala = fabsf(x - media) > UMBRAL_DESVIACIONES * desv;
  }
  ventanaIRH[idxVentana] = x;
  idxVentana = (idxVentana + 1) % TAM_VENTANA_ANOMALIA;
  if (nVentana < TAM_VENTANA_ANOMALIA) nVentana++;
  return anomala;
}

/* --- Tasa de descenso de nivel: regresion lineal por minimos
       cuadrados sobre los ultimos 60 s (1 muestra cada 2 s). --- */
const uint8_t  N_NIVEL = 30;
const uint32_t PASO_NIVEL_MS = 2000;
float   bufNivel[N_NIVEL];
uint8_t idxNivel = 0;
uint8_t nNivel   = 0;

void agregarNivel(float nivel) {
  if (isnan(nivel)) { nNivel = 0; idxNivel = 0; return; }   /* hueco -> reiniciar */
  bufNivel[idxNivel] = nivel;
  idxNivel = (idxNivel + 1) % N_NIVEL;
  if (nNivel < N_NIVEL) nNivel++;
}

float calcularDescenso() {
  if (nNivel < 8) return NAN;
  float sx = 0, sy = 0, sxy = 0, sxx = 0;
  uint8_t inicio = (idxNivel + N_NIVEL - nNivel) % N_NIVEL;
  for (uint8_t i = 0; i < nNivel; i++) {
    float x = i * (PASO_NIVEL_MS / 1000.0f);        /* segundos */
    float y = bufNivel[(inicio + i) % N_NIVEL];
    sx += x; sy += y; sxy += x * y; sxx += x * x;
  }
  float den = nNivel * sxx - sx * sx;
  if (den == 0) return NAN;
  float pendiente = (nNivel * sxy - sx * sy) / den;  /* %/s */
  return -pendiente * 60.0f;                          /* %/min, + = baja */
}

/* --- Presion de vapor de saturacion (Tetens), kPa --- */
float presionSaturacion(float tC) {
  return 0.6108f * expf(17.27f * tC / (tC + 237.3f));
}

/* --- Indice de evaporacion potencial (0-100) ---
   Basado en la ley de Dalton: la evaporacion es proporcional a
   la diferencia entre la presion de vapor en la superficie del
   agua y la del aire. Se combina con radiacion (fotoresistencia
   como proxy), temperatura ambiente y presion atmosferica.
   Normalizado sobre los sensores disponibles (patron de v7).
     Deficit de vapor (Dalton)  45
     Radiacion (luminosidad)    30
     Temperatura ambiente       15
     Presion atmosferica        10
   Es un indice relativo, no una tasa en mm/dia. */
void calcularEvaporacion(const Lecturas& L, Analitica& A) {
  float puntos = 0, maximo = 0;
  A.dpv = NAN;

  if (L.dhtOK && !isnan(L.tAmb) && !isnan(L.humedad)) {
    float tSup = (L.dsOK && !isnan(L.tAgua)) ? L.tAgua : L.tAmb;
    float ea   = presionSaturacion(L.tAmb) * L.humedad / 100.0f;
    float dpv  = presionSaturacion(tSup) - ea;
    if (dpv < 0) dpv = 0;
    A.dpv = dpv;
    puntos += constrain(dpv / 2.0f * 100.0f, 0, 100) * 0.45f;   /* 2 kPa = maximo */
    maximo += 45;

    puntos += constrain(L.tAmb / 35.0f * 100.0f, 0, 100) * 0.15f;
    maximo += 15;
  }

  if (!isnan(L.luminosidad)) {
    puntos += L.luminosidad * 0.30f;
    maximo += 30;
  }

  /* Menor presion -> el vapor escapa mas facil. 1013 hPa = 0,
     700 hPa = 100. En la Sabana (~750 hPa) aporta un valor
     casi constante que baja cuando entra un frente humedo. */
  if (L.bmpOK && !isnan(L.presion)) {
    puntos += constrain((1013.0f - L.presion) / 313.0f * 100.0f, 0, 100) * 0.10f;
    maximo += 10;
  }

  A.evap = (maximo == 0) ? NAN : constrain(puntos / maximo * 100.0f, 0, 100);
}

/* --- IRH: fusion multisensor ---
   Puntaje por variable, normalizado sobre las disponibles:
     Nivel                 40  (<15% 40, <30% 25, >95% 15)
     Tasa de descenso      20  (>=crit 20, >=umbral 10)
     Evaporacion           15  (ALTA 15, MEDIA 7)
     Temperatura del agua  10
     Humedad baja          15
   Reglas de fusion (se suman despues de normalizar):
     R1 descenso anomalo + calor extremo      +20
     R2 nivel bajo (<30%) + evaporacion ALTA  +10
   Anomalias estadisticas: +5 c/u (max 4). */
void calcularIRH(const Lecturas& L, Analitica& A) {
  float puntos = 0, maximo = 0;
  A.sensoresDatos = 0;

  if (L.hcOK) A.sensoresDatos++;
  if (L.hxOK) A.sensoresDatos++;

  if (L.nivelOK && !isnan(L.nivel)) {
    maximo += 40;
    if (L.nivel < 15)       puntos += 40;
    else if (L.nivel < 30)  puntos += 25;
    else if (L.nivel > 95)  puntos += 15;

    if (!isnan(A.descenso)) {
      maximo += 20;
      if (A.descenso >= UMBRAL_DESCENSO_CRIT) puntos += 20;
      else if (A.descenso >= UMBRAL_DESCENSO) puntos += 10;
    }
  }

  if (!isnan(A.evap)) {
    maximo += 15;
    if (A.evap >= EVAP_ALTA)       puntos += 15;
    else if (A.evap >= EVAP_MEDIA) puntos += 7;
  }

  if (L.dsOK && !isnan(L.tAgua)) {
    A.sensoresDatos++;
    maximo += 10;
    if (L.tAgua >= T_AGUA_ALTA)       puntos += 10;
    else if (L.tAgua >= T_AGUA_MEDIA) puntos += 5;
  }

  if (L.dhtOK && !isnan(L.humedad)) {
    A.sensoresDatos++;
    maximo += 15;
    if (L.humedad < HUMEDAD_BAJA)       puntos += 15;
    else if (L.humedad < HUMEDAD_MEDIA) puntos += 7;
  }

  if (L.bmpOK) A.sensoresDatos++;

  if (maximo == 0 || A.sensoresDatos == 0) {
    A.irhBase = 0; A.irh = 0;
    A.reglaDescensoCalor = false; A.reglaNivelEvap = false;
    return;
  }

  A.irhBase = puntos / maximo * 100.0f;

  bool calor = (L.dhtOK && L.tAmb >= T_AMB_CALOR) || (!isnan(A.evap) && A.evap >= EVAP_ALTA);
  A.reglaDescensoCalor = !isnan(A.descenso) && A.descenso >= UMBRAL_DESCENSO && calor;
  A.reglaNivelEvap = L.nivelOK && L.nivel < 30 && !isnan(A.evap) && A.evap >= EVAP_ALTA;

  if (A.reglaDescensoCalor) A.irhBase += 20;
  if (A.reglaNivelEvap)     A.irhBase += 10;
  A.irhBase = constrain(A.irhBase, 0, 100);

  bool anomala = detectarAnomalia(A.irhBase);
  if (anomala && A.anomalias < MAX_ANOMALIAS) A.anomalias++;
  else if (!anomala && A.anomalias > 0)       A.anomalias--;

  A.irh = constrain(A.irhBase + A.anomalias * 5.0f, 0, 100);
}

/* --- Maquina de estados (misma logica de v7) --- */
uint32_t inicioCondicionCritica = 0;

EstadoSistema calcularEstado(const Lecturas& L, const Analitica& A, EstadoSistema actual) {
  if (A.sensoresDatos == 0) { inicioCondicionCritica = 0; return ESTADO_ERROR; }

  bool confiables = L.nivelOK && (L.dsOK || L.dhtOK);
  if (!confiables) { inicioCondicionCritica = 0; return ESTADO_DEGRADADO; }

  if (A.irh >= UMBRAL_CRITICO) {
    if (inicioCondicionCritica == 0) inicioCondicionCritica = millis();
  } else if (!(actual == ESTADO_CRITICO && A.irh >= UMBRAL_SALIDA_CRITICO)) {
    inicioCondicionCritica = 0;
  }

  bool critico = (inicioCondicionCritica != 0) &&
                 (millis() - inicioCondicionCritica >= TIEMPO_CONFIRMACION_CRITICO);
  if (actual == ESTADO_CRITICO && A.irh >= UMBRAL_SALIDA_CRITICO) critico = true;

  if (critico) return ESTADO_CRITICO;
  if (A.irh >= UMBRAL_ALERTA || inicioCondicionCritica != 0) return ESTADO_ALERTA;
  return ESTADO_NORMAL;
}

void imprimirSerial(const Lecturas& L, const Analitica& A) {
  Serial.printf("\n--- WATER GUARD | %s | IRH %.1f (base %.1f) | anom %d ---\n",
                textoEstado(A.estado), A.irh, A.irhBase, A.anomalias);
  Serial.printf("Nivel %.1f%% (US %.1f%% | presion %.1f%%, %.1fcm)%s  Dist %.1fcm  Descenso %.2f%%/min\n",
                L.nivel, L.nivelUS, L.nivelP, L.colAgua, L.discrepanciaNivel ? " DISCREPANCIA" : "",
                L.distancia, A.descenso);
  Serial.printf("HX crudo %ld (offset %ld)\n", L.hxCrudo, HX_OFFSET);
  Serial.printf("T.agua %.1fC  T.amb %.1fC  Hum %.1f%%  Pres %.1fhPa\n", L.tAgua, L.tAmb, L.humedad, L.presion);
  Serial.printf("Luz %s  Lum %.1f%%  Evap %.1f (%s)  DPV %.2fkPa\n",
                L.luz ? "SI" : "NO", L.luminosidad, A.evap, textoEvaporacion(A.evap), A.dpv);
  Serial.printf("Sensores %u/5 [HC %d HX %d DHT %d DS %d BMP %d]  R1 %d R2 %d  Silencio %d  WiFi %s\n",
                A.sensoresDatos, L.hcOK, L.hxOK, L.dhtOK, L.dsOK, L.bmpOK,
                A.reglaDescensoCalor, A.reglaNivelEvap, alarmaSilenciada,
                WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "desconectado");
  /* Linea CSV para analisis offline (K-Means, graficas de la wiki) */
  Serial.printf("LOG,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%d,%d\n",
                millis(), L.nivel, L.nivelUS, L.nivelP, L.tAgua, L.tAmb, L.humedad, L.presion, L.luminosidad,
                A.evap, A.dpv, A.descenso, A.irh, A.anomalias, (int)A.estado);
}

/* ============================================================
   TAREA DE ANALITICA  (nucleo 1, prioridad 2)
   ============================================================ */

void tareaAnalitica(void* param) {

  Lecturas prevL;
  bool primera = true;
  uint32_t ultimoNivel = 0, ultimoHist = 0, ultimoSerial = 0;
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    uint32_t ahora = millis();

    /* Copia local de las lecturas (seccion critica corta) */
    xSemaphoreTake(mtxDatos, portMAX_DELAY);
    Lecturas L = lect;
    Analitica A = ana;
    xSemaphoreGive(mtxDatos);

    if (ahora - ultimoNivel >= PASO_NIVEL_MS) {
      ultimoNivel = ahora;
      agregarNivel(L.nivelOK ? L.nivel : NAN);
    }
    A.descenso = L.nivelOK ? calcularDescenso() : NAN;

    calcularEvaporacion(L, A);
    calcularIRH(L, A);

    EstadoSistema anterior = A.estado;
    A.estado = calcularEstado(L, A, anterior);
    A.verificandoCritico = (inicioCondicionCritica != 0 && A.estado != ESTADO_CRITICO);

    xSemaphoreTake(mtxDatos, portMAX_DELAY);

    bool r1Antes = ana.reglaDescensoCalor;
    bool r2Antes = ana.reglaNivelEvap;
    ana = A;

    char buf[72];

    /* Cambios de estado -> eventos */
    if (A.estado != anterior) {
      snprintf(buf, sizeof(buf), "Estado: %s -> %s (IRH %.0f)",
               textoEstado(anterior), textoEstado(A.estado), A.irh);
      uint8_t tipo = (A.estado == ESTADO_CRITICO || A.estado == ESTADO_ERROR) ? EV_CRITICO :
                     (A.estado == ESTADO_ALERTA || A.estado == ESTADO_DEGRADADO) ? EV_ALERTA : EV_INFO;
      registrarEventoSinLock(tipo, buf);
    }
    if (A.reglaDescensoCalor && !r1Antes) {
      snprintf(buf, sizeof(buf), "Descenso anomalo con calor: %.1f %%/min", A.descenso);
      registrarEventoSinLock(EV_ALERTA, buf);
    }
    if (A.reglaNivelEvap && !r2Antes) {
      registrarEventoSinLock(EV_ALERTA, "Nivel bajo con evaporacion ALTA");
    }

    /* Salud de sensores -> eventos */
    if (!primera) {
      if (L.hcOK  != prevL.hcOK)  registrarEventoSinLock(L.hcOK  ? EV_INFO : EV_ALERTA, L.hcOK  ? "HC-SR04 recuperado" : "HC-SR04 sin lectura");
      if (L.hxOK  != prevL.hxOK)  registrarEventoSinLock(L.hxOK  ? EV_INFO : EV_ALERTA, L.hxOK  ? "MD-PS002 recuperado" : "MD-PS002 sin lectura");
      if (L.discrepanciaNivel && !prevL.discrepanciaNivel) {
        snprintf(buf, sizeof(buf), "Discrepancia de nivel: US %.0f%% vs presion %.0f%%", L.nivelUS, L.nivelP);
        registrarEventoSinLock(EV_ALERTA, buf);
      }
      if (L.dhtOK != prevL.dhtOK) registrarEventoSinLock(L.dhtOK ? EV_INFO : EV_ALERTA, L.dhtOK ? "DHT11 recuperado"   : "DHT11 sin lectura");
      if (L.dsOK  != prevL.dsOK)  registrarEventoSinLock(L.dsOK  ? EV_INFO : EV_ALERTA, L.dsOK  ? "DS18B20 recuperado" : "DS18B20 sin lectura");
      if (L.bmpOK != prevL.bmpOK) registrarEventoSinLock(L.bmpOK ? EV_INFO : EV_ALERTA, L.bmpOK ? "BMP280 recuperado"  : "BMP280 sin lectura");
    }
    prevL = L;
    primera = false;

    /* Boton fisico (bandera puesta por la ISR) */
    if (botonPresionado) {
      botonPresionado = false;
      if (alarmaSonando(A.estado)) {
        alarmaSilenciada = true;
        silencioDesdeMs = ahora;
        registrarEventoSinLock(EV_INFO, "Alarma silenciada con el boton fisico");
      }
    }

    /* Rearme automatico del silencio */
    if (alarmaSilenciada) {
      bool sinPeligro = (A.estado != ESTADO_CRITICO && A.estado != ESTADO_ERROR);
      if (sinPeligro || ahora - silencioDesdeMs >= SILENCIO_MAX_MS) {
        alarmaSilenciada = false;
        registrarEventoSinLock(EV_INFO, "Alarma rearmada automaticamente");
      }
    }

    /* Historico para el tablero */
    if (ahora - ultimoHist >= PASO_HIST_MS) {
      ultimoHist = ahora;
      uint16_t pos = (histInicio + histCount) % N_HIST;
      if (histCount == N_HIST) histInicio = (histInicio + 1) % N_HIST;
      else histCount++;
      hist[pos] = { ahora / 1000, L.nivel, L.tAgua, L.tAmb, L.humedad,
                    L.presion, L.luminosidad, A.evap, A.irh, L.colAgua };
    }

    xSemaphoreGive(mtxDatos);

    if (ahora - ultimoSerial >= INTERVALO_SERIAL_MS) {
      ultimoSerial = ahora;
      imprimirSerial(L, A);
    }

    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_ANALITICA_MS));
  }
}


/* ============================================================
   TAREA DE SALIDAS: LEDS, BUZZER Y OLED  (nucleo 1, prio 1)
   ============================================================ */

void oledLinea(uint8_t fila, const char* texto) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", texto);   /* rellena para borrar restos */
  oled.drawString(0, fila, buf);
}

/* Filas 0 y 1 de la pantalla estan danadas: se usan 2 a 7. */
void actualizarOLED(const Lecturas& L, const Analitica& A, uint8_t pagina) {
  char b[48];   /* oledLinea() recorta a 16 columnas */

  if (A.estado == ESTADO_ERROR) {
    bool parpadeo = ((millis() / 500) % 2) == 0;
    oledLinea(2, parpadeo ? " *** ERROR ***" : "");
    oledLinea(3, " SIN SENSORES");
    oledLinea(4, " HC-SR04 : FALLA");
    oledLinea(5, " DHT11   : FALLA");
    oledLinea(6, " DS18B20 : FALLA");
    oledLinea(7, alarmaSilenciada ? " Silenciada" : " Revisar cables");
    return;
  }

  if (pagina == 0) {
    snprintf(b, sizeof(b), "Estado:%s", textoEstado(A.estado));
    oledLinea(2, b);
    snprintf(b, sizeof(b), "Anomalia: %s", A.anomalias > 0 ? "SI" : "NO");
    oledLinea(3, b);
    snprintf(b, sizeof(b), "Niv:%s%% Ag:%sC",
             isnan(L.nivel) ? "--" : String(L.nivel, 0).c_str(),
             isnan(L.tAgua) ? "--" : String(L.tAgua, 0).c_str());
    oledLinea(4, b);
    snprintf(b, sizeof(b), "Amb:%sC Hum:%s%%",
             isnan(L.tAmb) ? "--" : String(L.tAmb, 0).c_str(),
             isnan(L.humedad) ? "--" : String(L.humedad, 0).c_str());
    oledLinea(5, b);
    if (isnan(A.evap)) snprintf(b, sizeof(b), "Evapora: --");
    else snprintf(b, sizeof(b), "Evap:%d%% %s", (int)A.evap, textoEvaporacion(A.evap));
    oledLinea(6, b);

    if (alarmaSilenciada)                   oledLinea(7, "Alarma silenc.");
    else if (A.estado == ESTADO_DEGRADADO)  oledLinea(7, "Sin confirmar");
    else if (A.verificandoCritico)          oledLinea(7, "Verificando...");
    else {
      snprintf(b, sizeof(b), "Sens:%u/5 %s", A.sensoresDatos, WiFi.isConnected() ? "WiFi" : "NoWiFi");
      oledLinea(7, b);
    }
  } else {
    oledLinea(2, "Tablero en:");
    oledLinea(3, WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "(sin red)");
    snprintf(b, sizeof(b), "Pres:%shPa", isnan(L.presion) ? "--" : String(L.presion, 0).c_str());
    oledLinea(4, b);
    snprintf(b, sizeof(b), "Baja:%s%%/min", isnan(A.descenso) ? "--" : String(A.descenso, 1).c_str());
    oledLinea(5, b);
    snprintf(b, sizeof(b), "US:%s P:%s%s",
             isnan(L.nivelUS) ? "--" : String(L.nivelUS, 0).c_str(),
             isnan(L.nivelP) ? "--" : String(L.nivelP, 0).c_str(),
             L.discrepanciaNivel ? " !" : "");
    oledLinea(6, b);
    snprintf(b, sizeof(b), "IRH:%d Estado:%c", (int)A.irh, textoEstado(A.estado)[0]);
    oledLinea(7, b);
  }
}

void tareaSalidas(void* param) {

  uint32_t ultimaOLED = 0, ultimaPagina = 0;
  uint8_t pagina = 0;
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    uint32_t ahora = millis();

    xSemaphoreTake(mtxDatos, portMAX_DELAY);
    Lecturas L = lect;
    Analitica A = ana;
    bool silencio = alarmaSilenciada;
    xSemaphoreGive(mtxDatos);

    bool rapido = ((ahora / 250) % 2) == 0;
    bool lento  = ((ahora / 600) % 2) == 0;
    bool verde = false, amarillo = false, rojo = false, buzzer = false;

    switch (A.estado) {
      case ESTADO_NORMAL:    verde = true; break;
      case ESTADO_ALERTA:    amarillo = true; break;
      case ESTADO_DEGRADADO: amarillo = lento; break;
      case ESTADO_CRITICO:   rojo = true; buzzer = !silencio; break;
      case ESTADO_ERROR:
      default:               rojo = rapido; buzzer = rapido && !silencio; break;
    }

    digitalWrite(PIN_LED_VERDE,    verde);
    digitalWrite(PIN_LED_AMARILLO, amarillo);
    digitalWrite(PIN_LED_ROJO,     rojo);
    if (BUZZER_PASIVO) {
      static bool sonandoAntes = false;
      if (buzzer && !sonandoAntes)      tone(PIN_BUZZER, FRECUENCIA_BUZZER);
      else if (!buzzer && sonandoAntes) noTone(PIN_BUZZER);
      sonandoAntes = buzzer;
    } else {
      digitalWrite(PIN_BUZZER, buzzer);
    }

    if (ahora - ultimaPagina >= INTERVALO_PAGINA_MS) {
      ultimaPagina = ahora;
      pagina = (pagina + 1) % 2;
    }

    if (ahora - ultimaOLED >= INTERVALO_OLED_MS) {
      ultimaOLED = ahora;
      xSemaphoreTake(mtxI2C, portMAX_DELAY);
      actualizarOLED(L, A, pagina);
      xSemaphoreGive(mtxI2C);
    }

    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_SALIDAS_MS));
  }
}


/* ============================================================
   SEGURIDAD DEL TABLERO
   ============================================================ */

/* --- SHA-256 compatible con mbedTLS 2.x (core 2.x) y 3.x (core 3.x) --- */
String sha256Hex(const String& entrada) {
  uint8_t out[32];
#if MBEDTLS_VERSION_MAJOR >= 3
  mbedtls_sha256((const uint8_t*)entrada.c_str(), entrada.length(), out, 0);
#else
  mbedtls_sha256_ret((const uint8_t*)entrada.c_str(), entrada.length(), out, 0);
#endif
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02x", out[i]);
  hex[64] = '\0';
  return String(hex);
}

/* Comparacion en tiempo constante (no filtra por tiempos) */
bool igualesTiempoConstante(const String& a, const String& b) {
  if (a.length() != b.length()) return false;
  uint8_t dif = 0;
  for (size_t i = 0; i < a.length(); i++) dif |= a[i] ^ b[i];
  return dif == 0;
}

/* --- Sesiones: token aleatorio de 128 bits atado a la IP --- */
/* (estructuras de sesion declaradas arriba, junto al estado global) */

String nuevoToken() {
  uint8_t r[16];
  esp_fill_random(r, sizeof(r));
  char hex[33];
  for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02x", r[i]);
  hex[32] = '\0';
  return String(hex);
}

String crearSesion(uint32_t ip) {
  uint8_t elegido = 0;
  uint32_t masVieja = UINT32_MAX;
  for (uint8_t i = 0; i < N_SESIONES; i++) {
    if (!sesiones[i].activa) { elegido = i; break; }
    if (sesiones[i].ultimoUso < masVieja) { masVieja = sesiones[i].ultimoUso; elegido = i; }
  }
  String tok = nuevoToken();
  strncpy(sesiones[elegido].token, tok.c_str(), 33);
  sesiones[elegido].ip = ip;
  sesiones[elegido].ultimoUso = millis();
  sesiones[elegido].activa = true;
  return tok;
}

String tokenDeCookie() {
  if (!server.hasHeader("Cookie")) return "";
  String c = server.header("Cookie");
  int i = c.indexOf("WGSESION=");
  if (i < 0) return "";
  i += 9;
  int f = c.indexOf(';', i);
  return (f < 0) ? c.substring(i) : c.substring(i, f);
}

int buscarSesion(const String& tok, uint32_t ip) {
  if (tok.length() != 32) return -1;
  for (uint8_t i = 0; i < N_SESIONES; i++) {
    if (!sesiones[i].activa) continue;
    if (millis() - sesiones[i].ultimoUso > DURACION_SESION_MS) { sesiones[i].activa = false; continue; }
    if (sesiones[i].ip == ip && igualesTiempoConstante(tok, String(sesiones[i].token))) return i;
  }
  return -1;
}

/* --- Bloqueo por intentos fallidos: 5 fallos -> 2 min --- */
/* (estructuras de bloqueo declaradas arriba, junto al estado global) */

Intentos* registroIntentos(uint32_t ip, bool crear) {
  for (uint8_t i = 0; i < N_BLOQUEOS; i++) if (intentos[i].ip == ip) return &intentos[i];
  if (!crear) return nullptr;
  uint8_t k = 0; uint32_t viejo = UINT32_MAX;
  for (uint8_t i = 0; i < N_BLOQUEOS; i++) {
    if (intentos[i].ip == 0) { k = i; break; }
    if (intentos[i].desde < viejo) { viejo = intentos[i].desde; k = i; }
  }
  intentos[k] = { ip, 0, millis() };
  return &intentos[k];
}

bool ipBloqueada(uint32_t ip) {
  Intentos* r = registroIntentos(ip, false);
  if (!r || r->fallos < MAX_INTENTOS) return false;
  if (millis() - r->desde > TIEMPO_BLOQUEO_MS) { r->fallos = 0; return false; }
  return true;
}

/* --- Capa 2: misma subred (y lista blanca opcional) --- */
bool clienteEnRedLocal(IPAddress ip) {
  if (!WiFi.isConnected()) return false;
  uint32_t mascara = (uint32_t)WiFi.subnetMask();
  if (((uint32_t)ip & mascara) != ((uint32_t)WiFi.localIP() & mascara)) return false;
  if (USAR_LISTA_BLANCA) {
    for (uint8_t i = 0; i < N_LISTA_BLANCA; i++) if (LISTA_BLANCA[i] == ip) return true;
    return false;
  }
  return true;
}

/* Registra rechazos sin inundar el log (1 cada 30 s por IP) */
uint32_t ultimoRechazoIp = 0, ultimoRechazoMs = 0;
void registrarRechazo(IPAddress ip, const char* motivo) {
  if ((uint32_t)ip == ultimoRechazoIp && millis() - ultimoRechazoMs < 30000) return;
  ultimoRechazoIp = (uint32_t)ip; ultimoRechazoMs = millis();
  char b[72];
  snprintf(b, sizeof(b), "Acceso rechazado %s: %s", ip.toString().c_str(), motivo);
  registrarEvento(EV_SEGURIDAD, b);
}

/* Filtro comun a TODAS las rutas (incluida la de login) */
bool filtroRed() {
  IPAddress ip = server.client().remoteIP();
  if (!clienteEnRedLocal(ip)) {
    registrarRechazo(ip, "fuera de la WLAN");
    server.send(403, "text/plain; charset=utf-8", "403 - Solo dispositivos autorizados de la red local.");
    return false;
  }
  return true;
}

/* Filtro de rutas protegidas: red + sesion valida */
bool autorizado(bool esApi) {
  if (!filtroRed()) return false;
  uint32_t ip = (uint32_t)server.client().remoteIP();
  int s = buscarSesion(tokenDeCookie(), ip);
  if (s < 0) {
    if (esApi) server.send(401, "application/json", "{\"error\":\"sesion\"}");
    else { server.sendHeader("Location", "/login"); server.send(302, "text/plain", ""); }
    return false;
  }
  sesiones[s].ultimoUso = millis();
  return true;
}

void cabecerasSeguridad() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Frame-Options", "DENY");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("Referrer-Policy", "no-referrer");
}


/* ============================================================
   PAGINAS WEB (autocontenidas: la WLAN puede no tener internet,
   asi que no se carga nada de CDNs; graficas en <canvas>)
   ============================================================ */

const char LOGIN_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Guard - Ingreso</title>
<style>
:root{--fondo:#e9eff0;--tinta:#16323a;--suave:#5b7279;--agua:#1f6f95;--linea:#c6d3d6;--papel:#fff;--rojo:#a8322a}
@media(prefers-color-scheme:dark){:root{--fondo:#0f1d22;--tinta:#e3edef;--suave:#93a9ae;--agua:#5fb3d9;--linea:#2a3f46;--papel:#16272d;--rojo:#e07b72}}
*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;background:var(--fondo);color:var(--tinta);
font:16px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:1rem}
form{background:var(--papel);border:1px solid var(--linea);border-radius:14px;padding:2rem;width:100%;max-width:360px}
h1{margin:0 0 .25rem;font-size:1.5rem}p{margin:0 0 1.5rem;color:var(--suave);font-size:.95rem}
label{display:block;font-size:.9rem;margin:.9rem 0 .3rem}
input{width:100%;padding:.7rem .8rem;border-radius:8px;border:1px solid var(--linea);background:var(--fondo);color:var(--tinta);font-size:1rem}
input:focus,button:focus{outline:3px solid var(--agua);outline-offset:2px}
button{margin-top:1.4rem;width:100%;padding:.8rem;border:0;border-radius:8px;background:var(--agua);color:#fff;font-size:1rem;font-weight:600;cursor:pointer}
.error{color:var(--rojo);font-size:.9rem;margin-top:1rem;min-height:1.2em}
</style></head><body>
<form method="POST" action="/login">
<h1>Water Guard</h1>
<p>Tablero de control del punto de monitoreo. Acceso solo para autoridades en la red local.</p>
<label for="u">Usuario</label><input id="u" name="usuario" autocomplete="username" required>
<label for="c">Clave</label><input id="c" name="clave" type="password" autocomplete="current-password" required>
<button type="submit">Ingresar</button>
<div class="error" id="e"></div>
</form>
<script>
const e=new URLSearchParams(location.search).get('e');
document.getElementById('e').textContent=e==='1'?'Usuario o clave incorrectos.':e==='2'?'Demasiados intentos. Espere 2 minutos.':'';
</script></body></html>)HTML";


const char DASH_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Guard - Tablero</title>
<style>
:root{--fondo:#e9eff0;--papel:#fff;--tinta:#16323a;--suave:#5b7279;--linea:#c6d3d6;
--agua:#1f6f95;--agua2:#7fb9d4;--ok:#2f7d52;--alerta:#b7791f;--critico:#a8322a;--degradado:#6b6f8a}
@media(prefers-color-scheme:dark){:root{--fondo:#0f1d22;--papel:#16272d;--tinta:#e3edef;--suave:#93a9ae;--linea:#2a3f46;
--agua:#5fb3d9;--agua2:#2d6a86;--ok:#5cc08a;--alerta:#e0a84a;--critico:#e07b72;--degradado:#a3a7c4}}
*{box-sizing:border-box}
body{margin:0;background:var(--fondo);color:var(--tinta);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;gap:1rem;padding:1rem 1.25rem;max-width:1100px;margin:auto}
header h1{margin:0;font-size:1.15rem}
header small{color:var(--suave)}
button{font:inherit;border-radius:8px;border:1px solid var(--linea);background:var(--papel);color:var(--tinta);padding:.5rem .8rem;cursor:pointer}
button:focus-visible{outline:3px solid var(--agua);outline-offset:2px}
main{max-width:1100px;margin:auto;padding:0 1.25rem 2rem;display:grid;gap:1rem;grid-template-columns:260px 1fr}
@media(max-width:760px){main{grid-template-columns:1fr}.tanque{height:220px}header{flex-wrap:wrap}}
header>div:last-child{display:flex;gap:.5rem}
.banda{grid-column:1/-1;border-radius:12px;padding:1rem 1.25rem;color:#fff;display:flex;flex-wrap:wrap;gap:.75rem 1.5rem;align-items:center;justify-content:space-between;background:var(--degradado);transition:background .4s}
.banda b{font-size:1.6rem;letter-spacing:.02em}
.banda .det{opacity:.92}
.banda button{background:rgba(255,255,255,.15);border-color:rgba(255,255,255,.5);color:#fff}
.caja{background:var(--papel);border:1px solid var(--linea);border-radius:12px;padding:1rem 1.1rem}
.caja h2{margin:0 0 .75rem;font-size:.95rem;color:var(--suave);font-weight:600}
/* Tanque: el elemento central del tablero */
.tanque{position:relative;height:300px;border:3px solid var(--tinta);border-top:0;border-radius:0 0 18px 18px;overflow:hidden;background:
repeating-linear-gradient(to top,transparent 0 29px,var(--linea) 29px 30px)}
.agua{position:absolute;left:0;right:0;bottom:0;height:0;background:linear-gradient(var(--agua2),var(--agua));transition:height 1.2s ease}
.agua::before{content:"";position:absolute;top:-6px;left:0;right:0;height:12px;background:var(--agua2);border-radius:50%;opacity:.8}
.pct{position:absolute;inset:0;display:grid;place-items:center;font-size:3.2rem;font-weight:700;color:var(--tinta);mix-blend-mode:normal;text-shadow:0 0 12px var(--papel)}
.marca{position:absolute;z-index:2;left:0;right:0;border-top:2px dashed var(--critico);font-size:.75rem;color:var(--critico);padding-left:.3rem}
.sub{margin-top:.6rem;color:var(--suave);font-size:.9rem}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:.6rem}
.var{border:1px solid var(--linea);border-radius:10px;padding:.6rem .75rem;cursor:pointer;background:none;text-align:left;color:inherit}
.var[aria-pressed=true]{border-color:var(--agua);box-shadow:inset 0 0 0 1px var(--agua)}
.var span{display:block;color:var(--suave);font-size:.82rem}
.var strong{font-size:1.35rem}
.var em{font-style:normal;font-size:.85rem;color:var(--suave)}
canvas{width:100%;height:220px;display:block}
.col{display:grid;gap:1rem;align-content:start}
ul{list-style:none;margin:0;padding:0;max-height:260px;overflow:auto}
li{padding:.45rem 0;border-bottom:1px solid var(--linea);font-size:.9rem;display:flex;gap:.6rem}
li time{color:var(--suave);white-space:nowrap;font-variant-numeric:tabular-nums}
li .p{width:.55rem;height:.55rem;border-radius:50%;margin-top:.45rem;flex:none}
.sens{display:flex;flex-wrap:wrap;gap:.4rem .9rem;font-size:.88rem}
.sens i{font-style:normal}
.off{color:var(--critico)}
@media(prefers-reduced-motion:reduce){.agua,.banda{transition:none}}
</style></head><body>
<header>
<div><h1>Water Guard</h1><small id="red">Conectando...</small></div>
<div><button id="sonido">Activar sonido</button> <button id="salir">Salir</button></div>
</header>
<main>
<section class="banda" id="banda" aria-live="assertive">
<div><b id="estado">...</b><div class="det" id="detalle"></div></div>
<button id="btnAlarma" hidden>Silenciar alarma</button>
</section>

<div class="col">
<section class="caja">
<h2>Nivel del tanque</h2>
<div class="tanque"><div class="marca" style="bottom:30%">30%</div><div class="marca" style="bottom:15%">15%</div>
<div class="agua" id="agua"></div><div class="pct" id="pct">--</div></div>
<div class="sub" id="descenso">Tasa de descenso: --</div>
</section>
<section class="caja"><h2>Sensores</h2><div class="sens" id="sens"></div></section>
</div>

<div class="col">
<section class="caja">
<h2>Variables en tiempo real (toque una para ver su histórico)</h2>
<div class="grid" id="vars"></div>
</section>
<section class="caja">
<h2 id="tituloGraf">Histórico reciente</h2>
<canvas id="graf" aria-label="Gráfica del histórico"></canvas>
</section>
<section class="caja">
<h2>Notificaciones</h2>
<ul id="eventos"></ul>
</section>
</div>
</main>
<script>
const VARS=[
 {k:'nivel',n:'Nivel (fusionado)',u:'%',d:0},{k:'colAgua',n:'Columna de agua',u:'cm',d:1},{k:'tAgua',n:'Temp. agua',u:'°C',d:1},{k:'tAmb',n:'Temp. ambiente',u:'°C',d:1},
 {k:'hum',n:'Humedad relativa',u:'%',d:0},{k:'pres',n:'Presión atm.',u:'hPa',d:1},{k:'lum',n:'Radiación (luz)',u:'%',d:0},
 {k:'evap',n:'Evaporación potencial',u:'/100',d:0},{k:'irh',n:'Riesgo hídrico (IRH)',u:'/100',d:0}];
const COLOR={NORMAL:'--ok',ALERTA:'--alerta',CRITICO:'--critico',DEGRADADO:'--degradado',ERROR:'--critico'};
const TIPO=['--suave','--alerta','--critico','--agua'];
let sel='nivel',hist=null,ultimoEv=0,sonido=false,up0=0,t0=Date.now();
const $=id=>document.getElementById(id);
const css=v=>getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const fmt=(v,d)=>v===null||v===undefined?'--':Number(v).toFixed(d);
const hora=s=>new Date(t0-(up0-s)*1000).toLocaleTimeString('es-CO',{hour:'2-digit',minute:'2-digit',second:'2-digit'});

$('vars').innerHTML=VARS.map(v=>`<button class="var" data-k="${v.k}" aria-pressed="${v.k===sel}"><span>${v.n}</span><strong id="v_${v.k}">--</strong> <em>${v.u}</em></button>`).join('');
$('vars').onclick=e=>{const b=e.target.closest('.var');if(!b)return;sel=b.dataset.k;
 document.querySelectorAll('.var').forEach(x=>x.setAttribute('aria-pressed',x.dataset.k===sel));dibujar();};

async function api(url,op){const r=await fetch(url,op);if(r.status===401){location.href='/login';throw 0;}return r.json();}

function beep(critico){if(!sonido)return;try{const c=new AudioContext(),o=c.createOscillator(),g=c.createGain();
 o.frequency.value=critico?1400:880;o.connect(g);g.connect(c.destination);g.gain.value=.15;o.start();o.stop(c.currentTime+(critico?.9:.3));}catch(e){}
 if(navigator.vibrate)navigator.vibrate(critico?[300,100,300]:200);}

async function actualizar(){
 let d;try{d=await api('/api/estado');}catch(e){$('red').textContent='Sin conexión con el equipo';return;}
 up0=d.up;t0=Date.now();
 if(hist)dibujar();
 $('red').textContent=`${d.ip} · señal ${d.rssi} dBm`;
 $('estado').textContent=d.est;
 $('banda').style.background=css(COLOR[d.est]||'--degradado');
 let det=`IRH ${fmt(d.irh,0)}/100`;
 if(d.verif)det+=' · verificando condición crítica';
 if(d.r1)det+=' · descenso anómalo con calor';
 if(d.r2)det+=' · nivel bajo y evaporación alta';
 if(d.disc)det+=' · los sensores de nivel no coinciden';
 if(d.anom>0)det+=` · ${d.anom} anomalía(s)`;
 if(d.sil)det+=' · alarma silenciada';
 $('detalle').textContent=det;
 const b=$('btnAlarma');
 b.hidden=!(d.sonando||d.sil);
 b.textContent=d.sil?'Reactivar alarma':'Silenciar alarma';
 b.dataset.accion=d.sil?'rearmar':'silenciar';
 const v=d.v;
 const nivel=v.nivel;$('agua').style.height=(nivel===null?0:nivel)+'%';$('pct').textContent=nivel===null?'--':Math.round(nivel)+'%';
 $('descenso').textContent='Ultrasonido '+fmt(v.nivelUS,0)+'% · presión '+fmt(v.nivelP,0)+'% · descenso '+(d.desc===null?'calculando…':fmt(d.desc,2)+' %/min')+' · déficit de vapor '+fmt(d.dpv,2)+' kPa';
 VARS.forEach(x=>{const val=x.k==='evap'?d.evap:x.k==='irh'?d.irh:v[x.k];$('v_'+x.k).textContent=fmt(val,x.d);});
 $('v_evap').textContent=fmt(d.evap,0)+' '+(d.evapTxt||'');
 $('sens').innerHTML=Object.entries(d.sens).map(([k,ok])=>`<i class="${ok?'':'off'}">${k}: ${ok?'OK':'falla'}</i>`).join('')+`<i>luz: ${v.luz?'sí':'no'}</i>`;
 // notificaciones
 const lista=d.ev.slice().reverse();
 $('eventos').innerHTML=lista.map(e=>`<li><span class="p" style="background:${css(TIPO[e.k])}"></span><time>${hora(e.t)}</time><span>${e.m}</span></li>`).join('')||'<li>Sin eventos todavía.</li>';
 const nuevos=d.ev.filter(e=>e.id>ultimoEv);
 if(ultimoEv&&nuevos.some(e=>e.k>=1))beep(nuevos.some(e=>e.k===2));
 if(d.ev.length)ultimoEv=Math.max(ultimoEv,...d.ev.map(e=>e.id));
}

async function cargarHistorial(){try{hist=await api('/api/historial');dibujar();}catch(e){}}

function dibujar(){
 const c=$('graf'),x=c.getContext('2d'),dpr=devicePixelRatio||1,W=c.clientWidth,H=c.clientHeight;
 c.width=W*dpr;c.height=H*dpr;x.scale(dpr,dpr);x.clearRect(0,0,W,H);
 const meta=VARS.find(v=>v.k===sel);$('tituloGraf').textContent=`Histórico reciente: ${meta.n} (${meta.u})`;
 if(!hist||!hist.t.length){x.fillStyle=css('--suave');x.fillText('Reuniendo datos…',10,20);return;}
 const ys=hist[sel],ts=hist.t,pts=[];ys.forEach((y,i)=>{if(y!==null)pts.push([ts[i],y]);});
 if(pts.length<2){x.fillStyle=css('--suave');x.fillText('Sin datos suficientes para esta variable.',10,20);return;}
 let mn=Math.min(...pts.map(p=>p[1])),mx=Math.max(...pts.map(p=>p[1]));if(mx-mn<1){mn-=1;mx+=1;}
 const pad=(mx-mn)*.1;mn-=pad;mx+=pad;
 const L=44,R=10,T=10,B=24,t1=ts[0],t2=ts[ts.length-1]||t1+1;
 const X=t=>L+(t-t1)/(t2-t1||1)*(W-L-R),Y=y=>T+(1-(y-mn)/(mx-mn))*(H-T-B);
 x.strokeStyle=css('--linea');x.fillStyle=css('--suave');x.font='12px system-ui';x.lineWidth=1;
 for(let i=0;i<=4;i++){const v=mn+(mx-mn)*i/4,yy=Y(v);x.beginPath();x.moveTo(L,yy);x.lineTo(W-R,yy);x.stroke();x.fillText(v.toFixed(meta.d),4,yy+4);}
 x.fillText(hora(t1),L,H-6);const s=hora(t2);x.fillText(s,W-R-x.measureText(s).width,H-6);
 x.strokeStyle=css('--agua');x.lineWidth=2;x.beginPath();pts.forEach((p,i)=>i?x.lineTo(X(p[0]),Y(p[1])):x.moveTo(X(p[0]),Y(p[1])));x.stroke();
 const u=pts[pts.length-1];x.fillStyle=css('--agua');x.beginPath();x.arc(X(u[0]),Y(u[1]),3.5,0,7);x.fill();
}

$('btnAlarma').onclick=async e=>{const a=e.target.dataset.accion;
 await api('/api/alarma',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'accion='+a});actualizar();};
$('sonido').onclick=()=>{sonido=!sonido;$('sonido').textContent=sonido?'Sonido activo':'Activar sonido';if(sonido)beep(false);};
$('salir').onclick=async()=>{await fetch('/logout',{method:'POST'});location.href='/login';};
addEventListener('resize',dibujar);
actualizar();cargarHistorial();setInterval(actualizar,2000);setInterval(cargarHistorial,10000);
</script></body></html>)HTML";


/* ============================================================
   HANDLERS HTTP
   ============================================================ */

String jnum(float v, uint8_t dec) {
  if (isnan(v) || isinf(v)) return "null";
  return String(v, (unsigned int)dec);
}

String jstr(const char* s) {   /* escape minimo para JSON */
  String o = "\"";
  for (const char* p = s; *p; p++) {
    if (*p == '"' || *p == '\\') o += '\\';
    if ((uint8_t)*p >= 0x20) o += *p;
  }
  return o + "\"";
}

void handleRaiz() {
  if (!autorizado(false)) return;
  cabecerasSeguridad();
  server.send(200, "text/html; charset=utf-8", DASH_HTML);
}

void handleLoginGet() {
  if (!filtroRed()) return;
  cabecerasSeguridad();
  server.send(200, "text/html; charset=utf-8", LOGIN_HTML);
}

void handleLoginPost() {
  if (!filtroRed()) return;
  IPAddress ipCli = server.client().remoteIP();
  uint32_t ip = (uint32_t)ipCli;

  if (ipBloqueada(ip)) {
    server.sendHeader("Location", "/login?e=2");
    server.send(302, "text/plain", "");
    return;
  }

  String u = server.arg("usuario");
  String c = server.arg("clave");
  bool ok = igualesTiempoConstante(u, String(USUARIO_TABLERO)) &&
            igualesTiempoConstante(sha256Hex(c), String(HASH_CLAVE_TABLERO));

  if (!ok) {
    Intentos* r = registroIntentos(ip, true);
    if (r->fallos < 255) r->fallos++;
    r->desde = millis();
    char b[72];
    snprintf(b, sizeof(b), "Clave incorrecta desde %s (%u)", ipCli.toString().c_str(), r->fallos);
    registrarEvento(EV_SEGURIDAD, b);
    server.sendHeader("Location", r->fallos >= MAX_INTENTOS ? "/login?e=2" : "/login?e=1");
    server.send(302, "text/plain", "");
    return;
  }

  Intentos* r = registroIntentos(ip, false);
  if (r) r->fallos = 0;

  String tok = crearSesion(ip);
  char b[72];
  snprintf(b, sizeof(b), "Ingreso al tablero desde %s", ipCli.toString().c_str());
  registrarEvento(EV_SEGURIDAD, b);

  server.sendHeader("Set-Cookie", "WGSESION=" + tok + "; Path=/; HttpOnly; SameSite=Strict; Max-Age=1800");
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

void handleLogout() {
  if (!filtroRed()) return;
  int s = buscarSesion(tokenDeCookie(), (uint32_t)server.client().remoteIP());
  if (s >= 0) sesiones[s].activa = false;
  server.sendHeader("Set-Cookie", "WGSESION=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleEstado() {
  if (!autorizado(true)) return;

  xSemaphoreTake(mtxDatos, portMAX_DELAY);
  Lecturas L = lect;
  Analitica A = ana;
  bool sil = alarmaSilenciada;
  Evento ev[N_EVENTOS];
  uint8_t n = evCount;
  for (uint8_t i = 0; i < n; i++) ev[i] = eventos[(evInicio + i) % N_EVENTOS];
  xSemaphoreGive(mtxDatos);

  String j;
  j.reserve(3000);
  j += "{\"up\":" + String(millis() / 1000);
  j += ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"est\":\"" + String(textoEstado(A.estado)) + "\"";
  j += ",\"irh\":" + jnum(A.irh, 1) + ",\"irhb\":" + jnum(A.irhBase, 1);
  j += ",\"anom\":" + String(A.anomalias);
  j += ",\"evap\":" + jnum(A.evap, 1) + ",\"evapTxt\":\"" + String(textoEvaporacion(A.evap)) + "\"";
  j += ",\"dpv\":" + jnum(A.dpv, 3) + ",\"desc\":" + jnum(A.descenso, 2);
  j += ",\"r1\":" + String(A.reglaDescensoCalor ? "true" : "false");
  j += ",\"r2\":" + String(A.reglaNivelEvap ? "true" : "false");
  j += ",\"verif\":" + String(A.verificandoCritico ? "true" : "false");
  j += ",\"sil\":" + String(sil ? "true" : "false");
  j += ",\"sonando\":" + String(((A.estado == ESTADO_CRITICO || A.estado == ESTADO_ERROR) && !sil) ? "true" : "false");
  j += ",\"n\":" + String(A.sensoresDatos);
  j += ",\"sens\":{\"HC-SR04\":" + String(L.hcOK ? "true" : "false") +
       ",\"DHT11\":" + String(L.dhtOK ? "true" : "false") +
       ",\"DS18B20\":" + String(L.dsOK ? "true" : "false") +
       ",\"MD-PS002\":" + String(L.hxOK ? "true" : "false") +
       ",\"BMP280\":" + String(L.bmpOK ? "true" : "false") + "}";
  j += ",\"disc\":" + String(L.discrepanciaNivel ? "true" : "false");
  j += ",\"v\":{\"nivel\":" + jnum(L.nivel, 1) + ",\"dist\":" + jnum(L.distancia, 1) +
       ",\"nivelUS\":" + jnum(L.nivelUS, 1) + ",\"nivelP\":" + jnum(L.nivelP, 1) +
       ",\"colAgua\":" + jnum(L.colAgua, 1) +
       ",\"tAgua\":" + jnum(L.tAgua, 2) + ",\"tAmb\":" + jnum(L.tAmb, 1) +
       ",\"hum\":" + jnum(L.humedad, 1) + ",\"pres\":" + jnum(L.presion, 1) +
       ",\"lum\":" + jnum(L.luminosidad, 1) + ",\"luz\":" + String(L.luz ? "true" : "false") + "}";
  j += ",\"ev\":[";
  for (uint8_t i = 0; i < n; i++) {
    if (i) j += ",";
    j += "{\"id\":" + String(ev[i].id) + ",\"t\":" + String(ev[i].t) +
         ",\"k\":" + String(ev[i].tipo) + ",\"m\":" + jstr(ev[i].msg) + "}";
  }
  j += "]}";

  cabecerasSeguridad();
  server.send(200, "application/json", j);
}

void handleHistorial() {
  if (!autorizado(true)) return;

  /* Copia local para no retener el mutex mientras se arma el JSON */
  static Muestra copia[N_HIST];
  xSemaphoreTake(mtxDatos, portMAX_DELAY);
  uint16_t n = histCount;
  for (uint16_t i = 0; i < n; i++) copia[i] = hist[(histInicio + i) % N_HIST];
  xSemaphoreGive(mtxDatos);

  String j;
  j.reserve(n * 60 + 200);
  j += "{\"paso\":" + String(PASO_HIST_MS / 1000) + ",\"t\":[";
  for (uint16_t i = 0; i < n; i++) { if (i) j += ","; j += String(copia[i].t); }

  auto serie = [&](const char* nombre, float Muestra::*campo, uint8_t dec) {
    j += "],\""; j += nombre; j += "\":[";
    for (uint16_t i = 0; i < n; i++) { if (i) j += ","; j += jnum(copia[i].*campo, dec); }
  };
  serie("nivel", &Muestra::nivel, 1);
  serie("tAgua", &Muestra::tAgua, 2);
  serie("tAmb",  &Muestra::tAmb, 1);
  serie("hum",   &Muestra::hum, 1);
  serie("pres",  &Muestra::pres, 1);
  serie("lum",   &Muestra::lum, 1);
  serie("evap",  &Muestra::evap, 1);
  serie("irh",   &Muestra::irh, 1);
  serie("colAgua", &Muestra::colAgua, 1);
  j += "]}";

  cabecerasSeguridad();
  server.send(200, "application/json", j);
}

void handleAlarma() {
  if (!autorizado(true)) return;
  String accion = server.arg("accion");
  IPAddress ip = server.client().remoteIP();
  char b[72];

  xSemaphoreTake(mtxDatos, portMAX_DELAY);
  if (accion == "silenciar" && alarmaSonando(ana.estado)) {
    alarmaSilenciada = true;
    silencioDesdeMs = millis();
    snprintf(b, sizeof(b), "Alarma silenciada desde el tablero (%s)", ip.toString().c_str());
    registrarEventoSinLock(EV_INFO, b);
  } else if (accion == "rearmar" && alarmaSilenciada) {
    alarmaSilenciada = false;
    snprintf(b, sizeof(b), "Alarma reactivada desde el tablero (%s)", ip.toString().c_str());
    registrarEventoSinLock(EV_INFO, b);
  }
  bool sil = alarmaSilenciada;
  xSemaphoreGive(mtxDatos);

  server.send(200, "application/json", String("{\"sil\":") + (sil ? "true" : "false") + "}");
}

void handleNoEncontrado() {
  if (!filtroRed()) return;
  server.send(404, "text/plain", "404");
}


/* ============================================================
   WIFI
   ============================================================ */

bool mdnsActivo = false;

void conectarWiFi(uint32_t esperaMaxMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NOMBRE_MDNS);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t = millis();
  while (!WiFi.isConnected() && millis() - t < esperaMaxMs) delay(250);
}

void revisarWiFi() {
  static uint32_t ultimo = 0;
  static bool estabaConectado = false;
  if (millis() - ultimo < 5000) return;
  ultimo = millis();

  bool conectado = WiFi.isConnected();
  if (conectado && !estabaConectado) {
    char b[72];
    snprintf(b, sizeof(b), "WiFi conectado: http://%s", WiFi.localIP().toString().c_str());
    registrarEvento(EV_INFO, b);
    if (mdnsActivo) MDNS.end();
    mdnsActivo = MDNS.begin(NOMBRE_MDNS);
    if (mdnsActivo) MDNS.addService("http", "tcp", 80);
  } else if (!conectado && estabaConectado) {
    registrarEvento(EV_ALERTA, "WiFi perdido: sigue la alerta local");
  }
  if (!conectado) WiFi.reconnect();
  estabaConectado = conectado;
}


/* ============================================================
   SETUP
   ============================================================ */

void setup() {
  Serial.begin(115200);
  delay(1000);   /* da tiempo a abrir el Monitor Serial */
  Serial.println("\n==== WATER GUARD v8 (ESP32) INICIANDO ====");

  mtxDatos = xSemaphoreCreateMutex();
  mtxI2C   = xSemaphoreCreateMutex();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_HX_OUT, INPUT_PULLUP);   /* si se desconecta queda en alto = "sin dato" */
  pinMode(PIN_HX_SCK, OUTPUT);
  digitalWrite(PIN_HX_SCK, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_FOTORESISTENCIA, ADC_11db);   /* rango 0-3.3 V */

  /* I2C + OLED */
  Wire.begin(PIN_SDA, PIN_SCL);

  /* Escaner I2C: ayuda a diagnosticar la OLED y el BMP280.
     OLED esperada en 0x3C (algunas en 0x3D); BMP280 en 0x76/0x77. */
  Serial.print("[I2C] Dispositivos encontrados:");
  uint8_t encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", dir); encontrados++; }
  }
  Serial.println(encontrados ? "" : " NINGUNO (revisar SDA->D21, SCL->D22, VCC, GND)");
  oled.begin();
  oled.setFont(u8x8_font_amstrad_cpc_extended_r);
  oled.clear();
  oled.drawString(1, 3, "WATER GUARD v8");
  oled.drawString(1, 5, "Iniciando...");

  /* Sensores */
  dht.begin();
  sensorAgua.begin();
  sensorAgua.setResolution(12);
  sensorAgua.setWaitForConversion(false);
  Serial.printf("[DS18B20] dispositivos: %u\n", sensorAgua.getDeviceCount());

  bmpPresente = bmp.begin(0x76) || bmp.begin(0x77);
  Serial.printf("[BMP280] %s\n", bmpPresente ? "OK" : "NO DETECTADO (0x76/0x77)");
  if (bmpPresente) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  /* MD-PS002 / HX710B: deteccion y tara (manguera fuera del agua) */
  {
    long crudo, suma = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 15; i++) {
      if (leerHX710B(crudo, 200)) { suma += crudo; n++; }
    }
    hxPresente = (n >= 5);
    if (hxPresente && HX_AUTO_TARA) HX_OFFSET = suma / n;
    Serial.printf("[MD-PS002] %s  offset=%ld\n", hxPresente ? "OK" : "NO DETECTADO", HX_OFFSET);
  }

  /* WiFi (el sistema funciona aunque no conecte: la alerta
     in situ nunca depende de la red) */
  oled.drawString(1, 6, "Conectando WiFi");
  conectarWiFi(15000);
  if (WiFi.isConnected()) {
    Serial.printf("[WiFi] conectado. Tablero: http://%s  (o http://%s.local)\n",
                  WiFi.localIP().toString().c_str(), NOMBRE_MDNS);
    mdnsActivo = MDNS.begin(NOMBRE_MDNS);
    if (mdnsActivo) MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("[WiFi] sin conexion, se reintentara en segundo plano");
  }
  oled.clear();

  /* Servidor web */
  const char* cabeceras[] = { "Cookie" };
  server.collectHeaders(cabeceras, 1);
  server.on("/",              HTTP_GET,  handleRaiz);
  server.on("/login",         HTTP_GET,  handleLoginGet);
  server.on("/login",         HTTP_POST, handleLoginPost);
  server.on("/logout",        HTTP_POST, handleLogout);
  server.on("/api/estado",    HTTP_GET,  handleEstado);
  server.on("/api/historial", HTTP_GET,  handleHistorial);
  server.on("/api/alarma",    HTTP_POST, handleAlarma);
  server.onNotFound(handleNoEncontrado);
  server.begin();

  /* Tareas FreeRTOS. WiFi y la pila TCP corren en el nucleo 0;
     la medicion y la logica en el nucleo 1. */
  xTaskCreatePinnedToCore(tareaSensores,  "tSensores",  4096, NULL, 3, &hSensores,  1);
  xTaskCreatePinnedToCore(tareaAnalitica, "tAnalitica", 6144, NULL, 2, &hAnalitica, 1);
  xTaskCreatePinnedToCore(tareaSalidas,   "tSalidas",   4096, NULL, 1, &hSalidas,   1);

  /* Interrupciones (despues de crear hSensores) */
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO),  isrEcho,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), isrBoton, FALLING);

  registrarEvento(EV_INFO, "Sistema iniciado");
}


/* ============================================================
   LOOP: solo atiende el servidor web y la reconexion WiFi.
   La medicion NO pasa por aqui.
   ============================================================ */

void loop() {
  server.handleClient();
  revisarWiFi();
  vTaskDelay(pdMS_TO_TICKS(2));
}
