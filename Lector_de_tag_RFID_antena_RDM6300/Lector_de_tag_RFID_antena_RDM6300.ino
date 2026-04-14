/*
  =================================================================================================
  PROYECTO:    Lector_de_tag_RFID_antena_RDM6300
  VERSIÓN:     1.7 Corrección de orden en setup(): config flash cargada antes de inicializar salidas
  TIPO:        Lectura y validación de tag RFID 125 KHz con memoria flash
  AUTOR:       Santos
  ASISTENCIA:  ChatGPT, Claude
  FECHA:       10 de abril de 2026
  LUGAR:       CDMX
  =================================================================================================

  DESCRIPCIÓN GENERAL
  -------------------------------------------------------------------------------------------------
  Este programa permite leer un tag RFID de 125 KHz usando un módulo RDM6300 conectado por UART
  a un ESP32-C3 Super Mini.

  FUNCIONES PRINCIPALES
  -------------------------------------------------------------------------------------------------
  - Leer tags RFID por UART.
  - Guardar un tag en memoria flash del ESP32 al presionar un botón.
  - Comparar el tag leído contra el tag guardado.
  - Encender LED verde si el tag coincide.
  - Encender LED rojo si el tag no coincide.
  - Mantener encendido el LED correspondiente mientras el tag siga presente en la antena.
  - Apagar los LEDs cuando el tag se retire.
  - Mantener una salida de confirmación continua por GPIO 10 al sistema central
    mientras el tag correcto siga presente.
  - Permitir cambiar por Serial si GPIO 10 debe ser activo en LOW o activo en HIGH.
  - Guardar en memoria flash la configuración de GPIO 10 con el comando SAVE.

  HARDWARE USADO
  -------------------------------------------------------------------------------------------------
  - ESP32-C3 Super Mini
  - Módulo lector RFID 125 KHz RDM6300 o compatible
  - Botón entre GPIO 4 y GND
  - LED verde en GPIO 1
  - LED rojo en GPIO 2
  - Salida al sistema central en GPIO 10

  CONEXIONES
  -------------------------------------------------------------------------------------------------
  RFID:
    TX del RDM6300  ---> RX del ESP32-C3
    RX del RDM6300  ---> TX del ESP32-C3 (opcional)
    GND             ---> GND común
    VCC             ---> Alimentación según el módulo

  BOTÓN:
    Un lado         ---> GPIO 4
    Otro lado       ---> GND

  LEDs:
    LED verde       ---> GPIO 1
    LED rojo        ---> GPIO 2

  AVISO AL SISTEMA CENTRAL:
    GPIO 10         ---> Entrada digital del Arduino Uno
    GND ESP32       ---> GND Arduino Uno

  CONFIGURACIÓN DE LEDs
  -------------------------------------------------------------------------------------------------
  Este código incluye una variable para definir el tipo de conexión de los LEDs:

    - LEDS_COMUN_A_VCC = true
        El LED enciende con LOW y apaga con HIGH
        (típico cuando el LED tiene su lado común conectado a VCC)

    - LEDS_COMUN_A_VCC = false
        El LED enciende con HIGH y apaga con LOW
        (típico cuando el LED tiene su lado común conectado a GND)

  En tu caso actual:
    - LED verde en GPIO 1
    - LED rojo en GPIO 2
    - ambos activos en LOW
  Por lo tanto:
    LEDS_COMUN_A_VCC = true

  SALIDA AL SISTEMA CENTRAL
  -------------------------------------------------------------------------------------------------
  La salida en GPIO 10 puede configurarse por Serial y guardarse en flash:

    Modo por defecto:
      - LOW  = válido
      - HIGH = reposo / no válido

    Modo alternativo:
      - HIGH = válido
      - LOW  = reposo / no válido

  COMANDOS POR SERIAL
  -------------------------------------------------------------------------------------------------
    LOW
      Configura GPIO 10 como activo en LOW (modo por defecto)

    HIGH
      Configura GPIO 10 como activo en HIGH

    SAVE
      Guarda en memoria flash la configuración actual de GPIO 10
      (sin SAVE, el cambio solo dura hasta el próximo reinicio)

    STATUS
      Muestra el estado actual de la configuración

  NOTAS IMPORTANTES
  -------------------------------------------------------------------------------------------------
  1. Si el RDM6300 entrega TX a 5V, NO conectar directo al RX del ESP32-C3.
     Usa divisor de voltaje o level shifter.

  2. El botón usa INPUT_PULLUP:
     - sin presionar = HIGH
     - presionado    = LOW

  3. El tag guardado queda almacenado en memoria flash del ESP32 aunque se reinicie o apague.

  4. La configuración de GPIO 10 también se guarda en flash con el comando SAVE.

  5. Al reiniciar, la configuración de GPIO 10 se carga desde flash ANTES de inicializar
     la salida, por lo que GPIO 10 arranca con el nivel correcto desde el primer momento,
     sin mandar señal falsa al sistema central durante el arranque.

  6. Para cableados largos hacia el sistema central:
     - compartir GND
     - usar preferentemente señal activa en LOW
     - usar par trenzado señal + GND
     - si hay mucho ruido, usar transistor u optoacoplador

  FORMATO TÍPICO DE LA TRAMA RFID
  -------------------------------------------------------------------------------------------------
    0x02 + 10 caracteres ID + 2 checksum + 0x03

  Este programa extrae automáticamente los primeros 10 caracteres del ID.
  =================================================================================================
*/

#include <HardwareSerial.h>
#include <Preferences.h>

// =================================================================================================
// BLOQUE ÚNICO DE VARIABLES Y CONFIGURACIÓN
// =================================================================================================

// ---------------------------------------------------------------------------------
// CONFIGURACIÓN GENERAL
// ---------------------------------------------------------------------------------
static const bool LEDS_COMUN_A_VCC = true;   // true = LED activo en LOW, false = LED activo en HIGH

// ---------------------------------------------------------------------------------
// CONFIGURACIÓN DE PINES
// ---------------------------------------------------------------------------------
static const int PIN_RFID_RX        = 20;    // RX del ESP32, recibe desde TX del RDM6300
static const int PIN_RFID_TX        = 21;    // TX del ESP32, opcional hacia RX del RDM6300
static const int PIN_BOTON_GUARDAR  = 4;     // Botón para guardar el tag
static const int PIN_LED_VERDE      = 1;     // LED verde: tag correcto
static const int PIN_LED_ROJO       = 2;     // LED rojo: tag incorrecto
static const int PIN_SALIDA_CEREBRO = 10;    // Señal continua hacia el sistema central

// ---------------------------------------------------------------------------------
// PARÁMETROS DE FUNCIONAMIENTO
// ---------------------------------------------------------------------------------
const unsigned long TIMEOUT_PRESENCIA_TAG_MS = 300;   // Si pasa este tiempo sin lectura, se asume que el tag se retiró
const unsigned long BLOQUEO_GUARDADO_MS      = 1200;  // Evita guardar varias veces seguidas el mismo tag

// ---------------------------------------------------------------------------------
// CONFIGURACIÓN VARIABLE POR SERIAL
// ---------------------------------------------------------------------------------
bool SALIDA_ACTIVA_EN_BAJO = true;   // true = válido en LOW (por defecto), false = válido en HIGH

// ---------------------------------------------------------------------------------
// OBJETOS GLOBALES
// ---------------------------------------------------------------------------------
HardwareSerial RFIDSerial(1);     // UART1 para el lector RFID
Preferences preferencias;         // Memoria flash interna del ESP32

// ---------------------------------------------------------------------------------
// VARIABLES DE TRABAJO
// ---------------------------------------------------------------------------------
String tramaRFID = "";            // Buffer donde se almacena la trama recibida
bool   leyendoTrama = false;      // Indica si actualmente se está recibiendo una trama válida

String tagGuardado = "";          // Tag guardado permanentemente en memoria flash
String tagActual   = "";          // Tag que actualmente está presente frente a la antena

unsigned long ultimaLecturaValidaMs = 0;   // Momento de la última trama válida recibida
unsigned long ultimoGuardadoMs      = 0;   // Momento del último guardado válido

// =================================================================================================
// FUNCIONES DE APOYO PARA LEDs
// =================================================================================================

int ledEncendido() {
  return LEDS_COMUN_A_VCC ? LOW : HIGH;
}

int ledApagado() {
  return LEDS_COMUN_A_VCC ? HIGH : LOW;
}

void apagarLeds() {
  digitalWrite(PIN_LED_VERDE, ledApagado());
  digitalWrite(PIN_LED_ROJO,  ledApagado());
}

void prenderLedVerde() {
  digitalWrite(PIN_LED_VERDE, ledEncendido());
  digitalWrite(PIN_LED_ROJO,  ledApagado());
}

void prenderLedRojo() {
  digitalWrite(PIN_LED_VERDE, ledApagado());
  digitalWrite(PIN_LED_ROJO,  ledEncendido());
}

// =================================================================================================
// FUNCIONES DE APOYO PARA SALIDA AL CEREBRO
// =================================================================================================

void ponerSalidaCerebroValida() {
  digitalWrite(PIN_SALIDA_CEREBRO, SALIDA_ACTIVA_EN_BAJO ? LOW : HIGH);
}

void ponerSalidaCerebroReposo() {
  digitalWrite(PIN_SALIDA_CEREBRO, SALIDA_ACTIVA_EN_BAJO ? HIGH : LOW);
}

void inicializarSalidaCerebro() {
  pinMode(PIN_SALIDA_CEREBRO, OUTPUT);
  ponerSalidaCerebroReposo();
}

// =================================================================================================
// FUNCIONES GENERALES
// =================================================================================================

bool botonPresionado() {
  return digitalRead(PIN_BOTON_GUARDAR) == LOW;
}

String leerTagGuardadoDeMemoria() {
  preferencias.begin("rfid", true);
  String tag = preferencias.getString("tag", "");
  preferencias.end();
  return tag;
}

void guardarTagEnMemoria(const String& tag) {
  preferencias.begin("rfid", false);
  preferencias.putString("tag", tag);
  preferencias.end();

  tagGuardado = tag;

  Serial.println();
  Serial.println("========================================");
  Serial.println("TAG GUARDADO EN MEMORIA FLASH");
  Serial.print("Nuevo tag guardado: ");
  Serial.println(tagGuardado);
  Serial.println("========================================");
  Serial.println();
}

bool leerConfigSalidaDeMemoria() {
  preferencias.begin("rfid", true);
  bool val = preferencias.getBool("salidaLow", true);   // por defecto: activo en LOW
  preferencias.end();
  return val;
}

void guardarConfigSalidaEnMemoria(bool activoEnBajo) {
  preferencias.begin("rfid", false);
  preferencias.putBool("salidaLow", activoEnBajo);
  preferencias.end();
  Serial.println("Configuracion de GPIO 10 guardada en memoria flash.");
}

void mostrarConfiguracionLeds() {
  Serial.println("Configuracion de LEDs:");
  if (LEDS_COMUN_A_VCC) {
    Serial.println("- Modo: comun a VCC");
    Serial.println("- Encendido con: LOW");
    Serial.println("- Apagado con: HIGH");
  } else {
    Serial.println("- Modo: comun a GND");
    Serial.println("- Encendido con: HIGH");
    Serial.println("- Apagado con: LOW");
  }
  Serial.println();
}

void mostrarConfiguracionSalidaCerebro() {
  Serial.println("Configuracion actual de GPIO 10:");
  if (SALIDA_ACTIVA_EN_BAJO) {
    Serial.println("- Modo: ACTIVO EN LOW");
    Serial.println("- LOW  = valido");
    Serial.println("- HIGH = reposo / no valido");
  } else {
    Serial.println("- Modo: ACTIVO EN HIGH");
    Serial.println("- HIGH = valido");
    Serial.println("- LOW  = reposo / no valido");
  }
  Serial.println();
}

void actualizarSalidasSegunTag() {
  if (tagActual == "") {
    apagarLeds();
    ponerSalidaCerebroReposo();
    return;
  }

  if (tagGuardado == "") {
    apagarLeds();
    ponerSalidaCerebroReposo();
    return;
  }

  if (tagActual == tagGuardado) {
    prenderLedVerde();
    ponerSalidaCerebroValida();
  } else {
    prenderLedRojo();
    ponerSalidaCerebroReposo();
  }
}

void procesarTag(const String& tagLeido) {
  tagActual = tagLeido;
  ultimaLecturaValidaMs = millis();

  static String ultimoTagReportado = "";

  if (tagLeido != ultimoTagReportado) {
    Serial.println("----- TAG DETECTADO -----");
    Serial.print("ID leido: ");
    Serial.println(tagLeido);

    if (tagGuardado != "") {
      Serial.print("Tag guardado: ");
      Serial.println(tagGuardado);

      if (tagLeido == tagGuardado) {
        Serial.println("Resultado: TAG CORRECTO / COINCIDE");
      } else {
        Serial.println("Resultado: TAG DIFERENTE / NO COINCIDE");
      }
    } else {
      Serial.println("No hay tag guardado todavia.");
    }

    Serial.println("-------------------------");
    Serial.println();

    ultimoTagReportado = tagLeido;
  }

  if (botonPresionado()) {
    unsigned long ahora = millis();

    if (ahora - ultimoGuardadoMs > BLOQUEO_GUARDADO_MS) {
      guardarTagEnMemoria(tagLeido);
      ultimoGuardadoMs = ahora;
    }
  }

  actualizarSalidasSegunTag();
}

// =================================================================================================
// COMANDOS POR SERIAL
// =================================================================================================

void procesarComandosSerial() {
  if (!Serial.available()) return;

  String comando = Serial.readStringUntil('\n');
  comando.trim();
  comando.toUpperCase();

  if (comando == "LOW") {
    SALIDA_ACTIVA_EN_BAJO = true;
    Serial.println();
    Serial.println("Comando recibido: LOW");
    Serial.println("GPIO 10 configurado como ACTIVO EN LOW.");
    Serial.println("(usa SAVE para guardar este cambio en flash)");
    actualizarSalidasSegunTag();
    Serial.println();
  }
  else if (comando == "HIGH") {
    SALIDA_ACTIVA_EN_BAJO = false;
    Serial.println();
    Serial.println("Comando recibido: HIGH");
    Serial.println("GPIO 10 configurado como ACTIVO EN HIGH.");
    Serial.println("(usa SAVE para guardar este cambio en flash)");
    actualizarSalidasSegunTag();
    Serial.println();
  }
  else if (comando == "SAVE") {
    guardarConfigSalidaEnMemoria(SALIDA_ACTIVA_EN_BAJO);
    Serial.println();
    Serial.println("Comando recibido: SAVE");
    Serial.print("Configuracion guardada: GPIO 10 activo en ");
    Serial.println(SALIDA_ACTIVA_EN_BAJO ? "LOW" : "HIGH");
    Serial.println();
  }
  else if (comando == "STATUS") {
    Serial.println();
    Serial.println("Comando recibido: STATUS");
    mostrarConfiguracionSalidaCerebro();
  }
  else if (comando.length() > 0) {
    Serial.println();
    Serial.print("Comando no reconocido: ");
    Serial.println(comando);
    Serial.println("Comandos disponibles:");
    Serial.println("- LOW");
    Serial.println("- HIGH");
    Serial.println("- SAVE");
    Serial.println("- STATUS");
    Serial.println();
  }
}

// =================================================================================================
// SETUP
// =================================================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BOTON_GUARDAR, INPUT_PULLUP);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);

  // -----------------------------------------------------------------------
  // IMPORTANTE: cargar configuración de flash ANTES de inicializar salidas.
  // Así GPIO 10 arranca con el nivel correcto desde el primer momento,
  // sin mandar señal falsa al sistema central durante el reinicio.
  // -----------------------------------------------------------------------
  tagGuardado           = leerTagGuardadoDeMemoria();
  SALIDA_ACTIVA_EN_BAJO = leerConfigSalidaDeMemoria();

  apagarLeds();
  inicializarSalidaCerebro();   // usa el valor correcto recién cargado de flash

  RFIDSerial.begin(9600, SERIAL_8N1, PIN_RFID_RX, PIN_RFID_TX);

  Serial.println();
  Serial.println("========================================================");
  Serial.println(" Lector_de_tag_RFID_antena_RDM6300");
  Serial.println(" ESP32-C3 Super Mini + RDM6300");
  Serial.println("========================================================");
  Serial.println();

  mostrarConfiguracionLeds();
  mostrarConfiguracionSalidaCerebro();

  Serial.println("Modo de uso:");
  Serial.println("1) Acerca un tag al lector.");
  Serial.println("2) Si presionas el boton en GPIO 4 mientras se lee, ese tag se guarda.");
  Serial.println("3) Si el tag leido coincide con el guardado, se enciende el LED verde.");
  Serial.println("4) Si el tag leido es distinto, se enciende el LED rojo.");
  Serial.println("5) GPIO 10 cambia segun su configuracion serial actual.");
  Serial.println("6) Cuando el tag se retira, ambos LEDs se apagan.");
  Serial.println();

  Serial.println("Comandos por serial:");
  Serial.println("- LOW    -> GPIO 10 activo en LOW");
  Serial.println("- HIGH   -> GPIO 10 activo en HIGH");
  Serial.println("- SAVE   -> guardar configuracion actual de GPIO 10 en flash");
  Serial.println("- STATUS -> mostrar configuracion actual");
  Serial.println();

  if (tagGuardado != "") {
    Serial.print("Tag guardado en memoria: ");
    Serial.println(tagGuardado);
  } else {
    Serial.println("Aun no hay tag guardado en memoria.");
  }

  Serial.println();
  Serial.println("Sistema listo. Acerca un tag al lector...");
  Serial.println();
}

// =================================================================================================
// LOOP PRINCIPAL
// =================================================================================================

void loop() {
  // -----------------------------------------------------------------------------------------------
  // 0. Revisar comandos recibidos por Serial
  // -----------------------------------------------------------------------------------------------
  procesarComandosSerial();

  // -----------------------------------------------------------------------------------------------
  // 1. Leer datos del lector RFID por UART
  // -----------------------------------------------------------------------------------------------
  while (RFIDSerial.available()) {
    char c = RFIDSerial.read();

    if (c == 0x02) {
      tramaRFID = "";
      leyendoTrama = true;
    }
    else if (c == 0x03 && leyendoTrama) {
      leyendoTrama = false;

      if (tramaRFID.length() >= 12) {
        String tagID = tramaRFID.substring(0, 10);
        procesarTag(tagID);
      } else {
        Serial.println("Trama incompleta o no valida.");
      }
    }
    else if (leyendoTrama) {
      tramaRFID += c;
    }
  }

  // -----------------------------------------------------------------------------------------------
  // 2. Detectar cuando el tag ya fue retirado de la antena
  // -----------------------------------------------------------------------------------------------
  if (tagActual != "") {
    unsigned long ahora = millis();

    if (ahora - ultimaLecturaValidaMs > TIMEOUT_PRESENCIA_TAG_MS) {
      Serial.print("Tag retirado de la antena: ");
      Serial.println(tagActual);
      Serial.println();

      tagActual = "";
      actualizarSalidasSegunTag();
    }
  }
}
