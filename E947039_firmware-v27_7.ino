/*
 * ============================================================
 * ITA-9000 Power Quality Analyzer (E947039)
 * ============================================================
 * Version      : v27.6
 * Date         : May 2026
 * Author       : Eng. Brayan Daniel Vazquez-Gasca
 * Institution  : Tecnológico Nacional de México / Instituto Tecnológico de Apizaco 
 * Department   : Division of Graduate Studies and Research 
 * Degree       : M.Sc. degree in mechatronic engineering
 * Advisor      : Dr. Roberto Morales Caporal
 *
 * Description  : IEC 61000-4-30 Class S power quality analyzer.
 *                Dual-path telemetry (WiFi / 4G LTE), SD local
 *                storage, FreeRTOS dual-core architecture.
 * ============================================================
 */

// ---- Includes ----
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <math.h>
#include "RTClib.h"
#include "ADE9000RegMap.h"
#include "ADE9000API.h"
#include "ADE9000CalibrationInputs.h"
#include <PCF8574.h>
#include "PMIC_BQ25896.h"
#include "esp_task_wdt.h"

#define MQTT_MAX_PACKET_SIZE 1024
#define TINY_GSM_MODEM_SIM7600

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino_MQTT_Client.h>
#include <functional>
#include <ThingsBoard.h>
#include <time.h>


// ---- Pin definitions ----
#define ADE9000_CS_PIN      13
#define ADE9000_RESET_PIN   33
#define PM_1_PIN            32
#define ISOLATOR_EN_PIN     25
#define SD_CS_PIN           26
#define DREADY_PIN          34
#define IRQ0_PIN            39
#define IRQ1_PIN            36
#define RELAY_PIN            4
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define BUZZER_PIN          14
#define INT_U3_PIN          35
#define PCF_U2_ADDR       0x21
#define PCF_U3_ADDR       0x26


// ---- Communication configuration ----
#define WIFI_SSID      "INFINITUMGS5K"
#define WIFI_SSID_1    "INFINITUMGS5K"
#define WIFI_PASS_1    "0shnTkYefnFVQ&"
#define WIFI_SSID_2    "GASCA"
#define WIFI_PASS_2    "PQM2026GSK"

#define TB_SERVER           "thingsboard.cloud"
#define TB_PORT             8883
#define TB_PORT_4G          1883
#define TB_TOKEN            "p6LW5YLK5UwmS3CMn9PH"
#define TB_TOPIC            "v1/devices/me/telemetry"

#define GSM_RX_PIN          16
#define GSM_TX_PIN          17
#define GSM_SLEEP_PIN       27
#define GSM_APN             "internet.itelcel.com"
#define GSM_BAUD            115200


#define NTP_SERVER1         "pool.ntp.org"
#define NTP_SERVER2         "time.nist.gov"
#define NTP_GMT_OFFSET_SEC  0        
#define NTP_DST_OFFSET_SEC  0

#define WIFI_CONNECT_TIMEOUT_MS    15000
#define MQTT_RECONNECT_INTERVAL_MS  5000
#define NTP_REFRESH_INTERVAL_MS    86400000UL
#define WIFI_FAIL_THRESHOLD        3           
#define WIFI_RECOVERY_INTERVAL_MS  30000UL     
#define GPS_PUBLISH_INTERVAL_MS   1800000UL
#define PENDING_FILE               "/PENDING.csv"

#define COMM_STATE_OK        0
#define COMM_STATE_WARN      1
#define COMM_STATE_DOWN      2
#define COMM_STATE_SD_ONLY   3


// ---- ADE9000 energy accumulator register addresses (45-bit: _LO + _HI) ----
#define ADDR_AWATTHR_HI_A   0x2E7
#define ADDR_AWATTHR_HI_B   0x323
#define ADDR_AWATTHR_HI_C   0x35F
#define ADDR_AVARHR_HI_A    0x2F1
#define ADDR_AVARHR_HI_B    0x32D
#define ADDR_AVARHR_HI_C    0x369
#define ADDR_AVAHR_HI_A     0x2FB
#define ADDR_AVAHR_HI_B     0x337
#define ADDR_AVAHR_HI_C     0x373
#define ADDR_AFWATTHR_HI    0x305
#define ADDR_BFWATTHR_HI    0x341
#define ADDR_CFWATTHR_HI    0x37D
#define ADDR_AFVARHR_HI     0x30F
#define ADDR_BFVARHR_HI     0x34B
#define ADDR_CFVARHR_HI     0x387
#define ADDR_AFVAHR_HI      0x319
#define ADDR_BFVAHR_HI      0x355
#define ADDR_CFVAHR_HI      0x391
#define ADDR_VPEAK          0x401
#define ADDR_IPEAK          0x400
#define ADDR_OIA            0x40A
#define ADDR_OIB            0x40B
#define ADDR_OIC            0x40C
#define ADDR_OISTATUS       0x48F
#define ADDR_PHSIGN         0x49D


// Active-low LEDs via PCF8574 open-drain outputs
#define LED_ON   LOW
#define LED_OFF  HIGH
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

enum LedU2 {
    LED_1P2W        = 0,
    LED_3P4W        = 1,
    LED_3P3W        = 2,
    LED_WIFI        = 3,
    LED_4G_LTE      = 4,
    LED_SD_CARD     = 5,
    LED_COMM_STATUS = 6,
    LED_LOCAL_SETUP = 7
};

enum PinU3 {
    BTN_SETUP      = 0,
    BTN_MODE       = 1,
    BTN_SYSTEM     = 2,
    BTN_START_STOP = 3,
    LED_STATUS     = 4,
    LED_ERROR      = 5,
    LED_PCF_STATUS = 6,
    LED_POWER_ON   = 7
};


// ---- System topology modes ----
#define SYS_MODE_1P2W  0
#define SYS_MODE_3P4W  1
#define SYS_MODE_3P3W  2

#define ACCMODE_1P2W  0x3100
#define ACCMODE_3P4W  0x3100
#define ACCMODE_3P3W  0x3190

#define COMM_BLINK_MS    100
#define COMM_BURST_MS   1000
#define COMM_PAUSE_MS   4000

#define BEEP_NONE        0
#define BEEP_ERROR       1
#define BEEP_MODE_CHANGE 2


#define BATT_LOW_VOLT        3.2f   
#define BATT_LOW_VOLT_RESET  3.4f   
#define BATT_UPDATE_MS               500
#define BATT_LOW_BEEP_INTERVAL_MS  60000


static const float LIPO_VOLT_TABLE[] = {3.00, 3.30, 3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.10, 4.15, 4.20};
static const float LIPO_PCT_TABLE[]  = {0.0,  5.0,  15.0, 30.0, 50.0, 65.0, 75.0, 85.0, 93.0, 97.0, 100.0};


// ---- IEC 61000-4-30 Class S parameters (60 Hz, Mexico) ----
#define GRID_FREQUENCY          60.0f


#define U_DIN_1P2W_3P4W         127.0f
#define U_DIN_3P3W              220.0f

#define SAG_THRESHOLD           (g_u_din * 0.90f)
#define SWELL_THRESHOLD         (g_u_din * 1.10f)
#define INTERRUPTION_THRESHOLD  (g_u_din * 0.05f)
#define SAG_HYSTERESIS          (g_u_din * 0.02f)


// Converts Vrms to ADE9000 24-bit half-cycle RMS register code
#define V_TO_HALF_RMS_CODE(v)  ((uint32_t)(((v) * SCALE_FACTOR_RMS) / CAL_VRMS_CC) & 0x00FFFFFFUL)


#define EVT_DIP_CYC    1
#define EVT_SWELL_CYC  1

#define WINDOWS_PER_150_180     15
#define WINDOWS_PER_10MIN       200
#define HALF_CYCLE_MS_60HZ      8


// ---- ADE9000 calibration constants (from ADE9000CalibrationInputs.h) ----
#define CAL_VRMS_CC         13.29737f
#define CAL_IRMS_CC          3.75827f
#define CAL_POWER_CC         6.70756f
#define CAL_ENERGY_CC        1.90792f   


// Override: EGY_LD_ACCUM=0 — accumulate continuously, do not overwrite each period
#undef  ADE9000_EP_CFG
#define ADE9000_EP_CFG  0x0001
#define SCALE_FACTOR_RMS  1000000.0f
#define SCALE_FACTOR_POWER  1000.0f

SPISettings adeSettings(5000000, MSBFIRST, SPI_MODE0);


PCF8574 pcfU2(PCF_U2_ADDR);
PCF8574 pcfU3(PCF_U3_ADDR);
PMIC_BQ25896 bq25896;

volatile uint8_t u2_state = 0xFF;
volatile uint8_t u3_state = 0xFF;


// ---- Global state flags ----
volatile bool    g_measuring       = false;
volatile bool    g_button_pressed  = false;
volatile uint8_t g_system_mode = SYS_MODE_3P4W;
volatile uint8_t g_comm_mode   = 0;


volatile float   g_batt_voltage    = 0.0f;
volatile uint8_t g_batt_percent    = 0;
volatile bool    g_batt_charging   = false;
volatile bool    g_batt_full       = false;
volatile bool    g_batt_on_battery = false;
volatile bool    g_batt_error      = false;


ADE9000Class ade9000;
RTC_DS3231 rtc;


struct Ten12VoltageRMSRegs   vltgTen12RMSRegs;
struct Ten12CurrentRMSRegs   curntTen12RMSRegs;
struct HalfVoltageRMSRegs    vltgHalfRMSRegs;
struct HalfCurrentRMSRegs    curntHalfRMSRegs;
struct VoltageRMSRegs        vltgRMSRegs;
struct CurrentRMSRegs        curntRMSRegs;
struct ActivePowerRegs       actvPowerRegs;
struct ReactivePowerRegs     rctvPowerRegs;
struct ApparentPowerRegs     appntPowerRegs;
struct PowerFactorRegs       powerFactorRegs;
struct PeriodRegs            periodRegs;
struct AngleRegs             angleRegs;
struct VoltageTHDRegs        vltgTHDRegs;
struct CurrentTHDRegs        curntTHDRegs;
struct FundVoltageRMSRegs    vltgFundRMSRegs;
struct FundCurrentRMSRegs    curntFundRMSRegs;
struct FundActivePowerRegs   actvFundPowerRegs;
struct FundReactivePowerRegs rctvFundPowerRegs;
struct FundApparentPowerRegs appntFundPowerRegs;


typedef struct {
    float vrms_a, vrms_b, vrms_c;
    float irms_a, irms_b, irms_c, irms_n;
    float pa_a, pa_b, pa_c;
    float pr_a, pr_b, pr_c;
    float papp_a, papp_b, papp_c;
    float pa_fund_a, pa_fund_b, pa_fund_c;
    float pr_fund_a, pr_fund_b, pr_fund_c;
    float papp_fund_a, papp_fund_b, papp_fund_c;
    float vrms_fund_a, vrms_fund_b, vrms_fund_c;
    float irms_fund_a, irms_fund_b, irms_fund_c;
    float pf_a, pf_b, pf_c;
    float freq_a, freq_b, freq_c;
    float thd_v_a, thd_v_b, thd_v_c;
    float temp_chip;
    float thd_i_a, thd_i_b, thd_i_c;
    float angle_va_vb, angle_vb_vc, angle_va_vc;
    float angle_va_ia, angle_vb_ib, angle_vc_ic;
    float angle_ia_ib, angle_ib_ic, angle_ia_ic;
    float vrms_half_a, vrms_half_b, vrms_half_c;
    uint32_t timestamp;
    bool event_flag;
} IEC_10_12_Record;

typedef struct {
    float vrms_a, vrms_b, vrms_c;
    float irms_a, irms_b, irms_c, irms_n;
    float pa_a, pa_b, pa_c;
    float pr_a, pr_b, pr_c;
    float papp_a, papp_b, papp_c;
    float pa_fund_a, pa_fund_b, pa_fund_c;
    float pr_fund_a, pr_fund_b, pr_fund_c;
    float papp_fund_a, papp_fund_b, papp_fund_c;
    float vrms_fund_a, vrms_fund_b, vrms_fund_c;
    float irms_fund_a, irms_fund_b, irms_fund_c;
    float pf_a, pf_b, pf_c;
    float freq_a, freq_b, freq_c;
    float thd_v_a, thd_v_b, thd_v_c;
    float thd_i_a, thd_i_b, thd_i_c;
    float temp_chip;
    float u2_voltage;
    float u2_current;
    uint64_t timestamp;  
    bool event_flag;
    uint8_t window_count;
    float angle_va_vb, angle_vb_vc, angle_va_vc;
    float angle_va_ia, angle_vb_ib, angle_vc_ic;
    float angle_ia_ib, angle_ib_ic, angle_ia_ic;
    float vrms_half_a, vrms_half_b, vrms_half_c;
} IEC_150_180_Record;

typedef struct {
    float vrms_a, vrms_b, vrms_c;
    float irms_a, irms_b, irms_c, irms_n;
    float pa_a, pa_b, pa_c;
    float pr_a, pr_b, pr_c;
    float papp_a, papp_b, papp_c;
    float pa_fund_a, pa_fund_b, pa_fund_c;
    float pr_fund_a, pr_fund_b, pr_fund_c;
    float papp_fund_a, papp_fund_b, papp_fund_c;
    float vrms_fund_a, vrms_fund_b, vrms_fund_c;
    float irms_fund_a, irms_fund_b, irms_fund_c;
    float pf_a, pf_b, pf_c;
    float freq_a;
    float thd_v_a, thd_v_b, thd_v_c;
    float thd_i_a, thd_i_b, thd_i_c;
    float temp_chip;
    float u2_voltage;
    float u2_current;
    uint32_t timestamp_start;
    uint32_t timestamp_end;
    bool event_flag;
    uint8_t window_count;
} IEC_10MIN_Record;

typedef enum {
    EVT_SAG = 0,
    EVT_SWELL,
    EVT_INTERRUPTION
} EventType;

typedef struct {
    EventType type;
    uint32_t  timestamp_start;
    uint32_t  timestamp_end;
    float     residual_voltage;
    float     peak_voltage;
    uint8_t   phase;
    uint32_t  duration_ms;
} IEC_Event;


// ---- FreeRTOS queue and mutex handles ----
QueueHandle_t xQueue10min;
QueueHandle_t xQueueEvents;
QueueHandle_t xQueue150Comm;
QueueHandle_t xQueueEventComm;
SemaphoreHandle_t xIsolatorMutex;


volatile uint8_t  g_comm_state      = COMM_STATE_SD_ONLY;
volatile bool     g_comm_success    = false;
volatile uint8_t  g_beep_request    = BEEP_NONE;
volatile bool     g_failover_active = false;
volatile bool     g_sd_error        = false;
volatile bool     g_led_error_sd    = false;
volatile bool     g_start_attrs_sent = false;


volatile bool     g_pending_flush_on_start = false;


volatile float   g_vpeak     = 0.0f;
volatile float   g_ipeak     = 0.0f;
volatile uint8_t g_oc_flag   = 0;
volatile uint8_t g_oc_phase  = 0;
volatile float   g_oc_peak_a = 0.0f;
volatile float   g_oc_peak_b = 0.0f;
volatile float   g_oc_peak_c = 0.0f;
volatile uint8_t g_phase_seq = 0;


volatile float   g_gps_lat    = 0.0f;
volatile float   g_gps_lon    = 0.0f;
volatile uint8_t g_gps_fix    = 0;
volatile float   g_gps_hdop   = 99.9f;
static   uint32_t g_gps_last_ms = 0;


// ---- Runtime voltage thresholds (updated by hmi_apply_system_mode) ----
volatile float g_v_nom = 127.0f;
volatile float g_u_din = 127.0f;  
volatile float g_v_min = 114.3f;
volatile float g_v_max = 139.7f;


// ---- Per-type event flags and data (cleared after publish) ----
volatile bool     g_evt_sag          = false;
volatile bool     g_evt_swell        = false;
volatile bool     g_evt_interruption = false;


volatile char     g_evt_sag_phase    = '-';
volatile uint32_t g_evt_sag_ts      = 0;
volatile uint32_t g_evt_sag_dur     = 0;
volatile float    g_evt_sag_res     = 0.0f;
volatile char     g_evt_int_phase    = '-';
volatile uint32_t g_evt_int_ts      = 0;
volatile uint32_t g_evt_int_dur     = 0;
volatile float    g_evt_int_res     = 0.0f;
volatile char     g_evt_swl_phase    = '-';
volatile uint32_t g_evt_swl_ts      = 0;
volatile uint32_t g_evt_swl_dur     = 0;
volatile float    g_evt_swl_peak    = 0.0f;


volatile bool g_alarm_batt_sent     = false;
volatile bool g_alarm_temp_sent     = false;
volatile bool g_alarm_failover_sent = false;


#ifndef ADDR_STATUS0
#define ADDR_STATUS0        0x402
#endif
#define STATUS0_RMS1012RDY  (1UL << 20)
#define STATUS0_RMSONERDY   (1UL << 19)
#define RMS1012_SYNC_TIMEOUT_MS  15

volatile bool irq1_fired      = false;
volatile bool irq0_rms_ready  = false;

static uint32_t g_boot_ms       = 0;
static uint32_t g_last_ntp_sync = 0;


// Millis-anchored UTC epoch — updated on every NTP/NITZ sync
static volatile uint64_t g_ts_epoch_ms      = 0;   
static volatile uint32_t g_ts_millis_anchor = 0;   
#define NTP_WARN_INTERVAL_S  86400UL

volatile uint32_t g_tx_messages   = 0;
volatile uint32_t g_tx_msg_wifi   = 0;
volatile uint32_t g_tx_msg_4g     = 0;
volatile uint64_t g_tx_bytes_wifi = 0ULL;
volatile uint64_t g_tx_bytes_4g   = 0ULL;


// Writes DIP_LVL and SWELL_LVL to ADE9000 based on current g_u_din.
// Must be called after g_u_din is updated (topology change, START).
static void setupHardwareEventThresholds() {
    uint32_t dip_code   = V_TO_HALF_RMS_CODE(g_u_din * 0.90f);
    uint32_t swell_code = V_TO_HALF_RMS_CODE(g_u_din * 1.10f);
    Serial.printf("[EVT-HW] Umbrales: DIP_LVL=0x%06X (%.2fV), SWELL_LVL=0x%06X (%.2fV)\n",
                  dip_code, g_u_din * 0.90f, swell_code, g_u_din * 1.10f);
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        digitalWrite(ISOLATOR_EN_PIN, HIGH);
        SPI.beginTransaction(adeSettings);
        ade9000.SPI_Write_32(ADDR_DIP_LVL,   dip_code);
        ade9000.SPI_Write_32(ADDR_SWELL_LVL, swell_code);
        
        ade9000.SPI_Write_32(ADDR_STATUS1,   0x03E00000UL);
        
        uint32_t rb_dip   = ade9000.SPI_Read_32(ADDR_DIP_LVL)   & 0x00FFFFFFUL;
        uint32_t rb_swell = ade9000.SPI_Read_32(ADDR_SWELL_LVL) & 0x00FFFFFFUL;
        uint32_t rb_mask1 = ade9000.SPI_Read_32(ADDR_MASK1);
        uint16_t rb_dipcyc   = ade9000.SPI_Read_16(ADDR_DIP_CYC);
        uint16_t rb_swellcyc = ade9000.SPI_Read_16(ADDR_SWELL_CYC);
        SPI.endTransaction();
        xSemaphoreGive(xIsolatorMutex);
        Serial.printf("[EVT-HW] Readback: DIP_LVL=0x%06X SWELL_LVL=0x%06X MASK1=0x%08X DIP_CYC=%u SWELL_CYC=%u\n",
                      rb_dip, rb_swell, rb_mask1, rb_dipcyc, rb_swellcyc);
        if (rb_dip != dip_code || rb_swell != swell_code)
            Serial.println("[EVT-HW] ERROR: readback no coincide — verificar SPI/ADE9000");
        else if (rb_mask1 != 0x03E20000UL)
            Serial.printf("[EVT-HW] WARN: MASK1=0x%08X esperado 0x03E20000 — DIP/SWELL IRQ1 puede no funcionar\n", rb_mask1);
        else
            Serial.println("[EVT-HW] OK — DIP/SWELL hardware configurado correctamente.");
    } else {
        Serial.println("[EVT-HW] ERROR: mutex no disponible para escribir umbrales.");
    }
}


// ---- Interrupt service routines ----
void IRAM_ATTR isr_dready() {}
void IRAM_ATTR isr_irq0()   { irq0_rms_ready = true; }
void IRAM_ATTR isr_irq1()   { irq1_fired = true; }


static float acc_150_vrms_a = 0, acc_150_vrms_b = 0, acc_150_vrms_c = 0;
static float acc_150_irms_a = 0, acc_150_irms_b = 0, acc_150_irms_c = 0, acc_150_irms_n = 0;
static float acc_150_pa_a = 0, acc_150_pa_b = 0, acc_150_pa_c = 0;
static float acc_150_pr_a = 0, acc_150_pr_b = 0, acc_150_pr_c = 0;
static float acc_150_papp_a = 0, acc_150_papp_b = 0, acc_150_papp_c = 0;
static float acc_150_pf_a = 0, acc_150_pf_b = 0, acc_150_pf_c = 0;
static float acc_150_freq = 0, acc_150_freq_b = 0, acc_150_freq_c = 0;
static float acc_150_thd_v_a = 0, acc_150_thd_v_b = 0, acc_150_thd_v_c = 0;
static float acc_150_thd_i_a = 0, acc_150_thd_i_b = 0, acc_150_thd_i_c = 0;
static float acc_150_pa_fund_a = 0, acc_150_pa_fund_b = 0, acc_150_pa_fund_c = 0;
static float acc_150_pr_fund_a = 0, acc_150_pr_fund_b = 0, acc_150_pr_fund_c = 0;
static float acc_150_papp_fund_a = 0, acc_150_papp_fund_b = 0, acc_150_papp_fund_c = 0;
static float acc_150_vrms_fund_a = 0, acc_150_vrms_fund_b = 0, acc_150_vrms_fund_c = 0;
static float acc_150_irms_fund_a = 0, acc_150_irms_fund_b = 0, acc_150_irms_fund_c = 0;
static float acc_150_temp = 0;
static int   cnt_150  = 0;
static bool  flag_150 = false;

static float acc_10m_vrms_a = 0, acc_10m_vrms_b = 0, acc_10m_vrms_c = 0;
static float acc_10m_irms_a = 0, acc_10m_irms_b = 0, acc_10m_irms_c = 0, acc_10m_irms_n = 0;
static float acc_10m_pa_a = 0, acc_10m_pa_b = 0, acc_10m_pa_c = 0;
static float acc_10m_pr_a = 0, acc_10m_pr_b = 0, acc_10m_pr_c = 0;
static float acc_10m_papp_a = 0, acc_10m_papp_b = 0, acc_10m_papp_c = 0;
static float acc_10m_pf_a = 0, acc_10m_pf_b = 0, acc_10m_pf_c = 0;
static float acc_10m_freq = 0;
static float acc_10m_thd_v_a = 0, acc_10m_thd_v_b = 0, acc_10m_thd_v_c = 0;
static float acc_10m_thd_i_a = 0, acc_10m_thd_i_b = 0, acc_10m_thd_i_c = 0;
static float acc_10m_pa_fund_a = 0, acc_10m_pa_fund_b = 0, acc_10m_pa_fund_c = 0;
static float acc_10m_pr_fund_a = 0, acc_10m_pr_fund_b = 0, acc_10m_pr_fund_c = 0;
static float acc_10m_papp_fund_a = 0, acc_10m_papp_fund_b = 0, acc_10m_papp_fund_c = 0;
static float acc_10m_vrms_fund_a = 0, acc_10m_vrms_fund_b = 0, acc_10m_vrms_fund_c = 0;
static float acc_10m_irms_fund_a = 0, acc_10m_irms_fund_b = 0, acc_10m_irms_fund_c = 0;
static float acc_10m_temp = 0;
static float acc_10m_u2v = 0, acc_10m_u2i = 0;
static int   cnt_10m  = 0;
static bool  flag_10m = false;
static uint32_t ts_10m_start = 0;


static bool  sag_active_a = false,   sag_active_b = false,   sag_active_c = false;
static bool  swell_active_a = false, swell_active_b = false, swell_active_c = false;
static bool  int_active_a = false,   int_active_b = false,   int_active_c = false;
static float sag_min_a = 999,  sag_min_b = 999,  sag_min_c = 999;
static float int_min_a = 999,  int_min_b = 999,  int_min_c = 999;  
static float swell_max_a = 0,  swell_max_b = 0,  swell_max_c = 0;

static uint32_t sag_ts_a = 0,   sag_ts_b = 0,   sag_ts_c = 0;
static uint32_t sag_ms_a = 0,   sag_ms_b = 0,   sag_ms_c = 0;
static uint32_t swell_ts_a = 0, swell_ts_b = 0, swell_ts_c = 0;
static uint32_t swell_ms_a = 0, swell_ms_b = 0, swell_ms_c = 0;
static uint32_t int_ts_a = 0,   int_ts_b = 0,   int_ts_c = 0;
static uint32_t int_ms_a = 0,   int_ms_b = 0,   int_ms_c = 0;


// ---- Time helpers ----
static char _timeStr[10];
const char* getTimeStr() {
    DateTime now = rtc.now();
    snprintf(_timeStr, sizeof(_timeStr), "%02d:%02d:%02d",
             now.hour(), now.minute(), now.second());
    return _timeStr;
}

uint32_t getUnixTS() {
    const uint32_t TS_MIN = 1704067200UL;
    const uint32_t TS_MAX = 2051222400UL;
    uint32_t ts = (uint32_t)rtc.now().unixtime();
    if (ts < TS_MIN || ts > TS_MAX) {
        Serial.printf("[WARN] getUnixTS: timestamp invalido %u — usando fallback millis\n", ts);
        return TS_MIN + (millis() / 1000UL);
    }
    return ts;
}


static uint64_t s_last_rec_ts = 0;  

// Returns a monotonically increasing UTC millisecond timestamp.
// Uses millis()-anchored epoch to avoid jumps from mid-session NTP corrections.
static uint64_t getRecordTs() {
    
    const uint64_t TS_MIN_MS = 1704067200000ULL;
    const uint64_t TS_MAX_MS = 1893456000000ULL;

    uint64_t ts_ms;
    if (g_ts_epoch_ms == 0 ||
        g_ts_epoch_ms < TS_MIN_MS ||
        g_ts_epoch_ms > TS_MAX_MS) {
        
        uint64_t rtc_ms = (uint64_t)getUnixTS() * 1000ULL;
        ts_ms = (rtc_ms >= TS_MIN_MS && rtc_ms <= TS_MAX_MS) ? rtc_ms : 0;
    } else {
        
        uint32_t elapsed = millis() - g_ts_millis_anchor;
        ts_ms = g_ts_epoch_ms + (uint64_t)elapsed;
        
        if (ts_ms < TS_MIN_MS || ts_ms > TS_MAX_MS) ts_ms = 0;
    }

    
    if (ts_ms == 0) return 0;

    
    if (ts_ms <= s_last_rec_ts) {
        ts_ms = s_last_rec_ts + 3000ULL;
    }
    s_last_rec_ts = ts_ms;
    return ts_ms;
}


void readAllADERegisters() {
    ade9000.ReadTen12VoltageRMSRegs(&vltgTen12RMSRegs);
    ade9000.ReadTen12CurrentRMSRegs(&curntTen12RMSRegs);
    ade9000.ReadHalfVoltageRMSRegs(&vltgHalfRMSRegs);
    ade9000.ReadHalfCurrentRMSRegs(&curntHalfRMSRegs);
    ade9000.ReadActivePowerRegs(&actvPowerRegs);
    ade9000.ReadReactivePowerRegs(&rctvPowerRegs);
    ade9000.ReadApparentPowerRegs(&appntPowerRegs);
    ade9000.ReadPowerFactorRegsnValues(&powerFactorRegs);
    ade9000.ReadPeriodRegsnValues(&periodRegs);
    ade9000.ReadAngleRegsnValues(&angleRegs);
    ade9000.ReadVoltageTHDRegsnValues(&vltgTHDRegs);
    ade9000.ReadCurrentTHDRegsnValues(&curntTHDRegs);
    ade9000.ReadFundVoltageRMSRegs(&vltgFundRMSRegs);
    ade9000.ReadFundCurrentRMSRegs(&curntFundRMSRegs);
    ade9000.ReadFundActivePowerRegs(&actvFundPowerRegs);
    ade9000.ReadFundReactivePowerRegs(&rctvFundPowerRegs);
    ade9000.ReadFundApparentPowerRegs(&appntFundPowerRegs);
}


IEC_10_12_Record convertToPhysical(uint32_t timestamp) {
    IEC_10_12_Record r;
    r.timestamp  = timestamp;
    r.event_flag = false;

    r.vrms_a = ((float)vltgTen12RMSRegs.Ten12VoltageRMSReg_A * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_b = ((float)vltgTen12RMSRegs.Ten12VoltageRMSReg_B * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_c = ((float)vltgTen12RMSRegs.Ten12VoltageRMSReg_C * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_a = ((float)curntTen12RMSRegs.Ten12CurrentRMSReg_A * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_b = ((float)curntTen12RMSRegs.Ten12CurrentRMSReg_B * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_c = ((float)curntTen12RMSRegs.Ten12CurrentRMSReg_C * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_n = ((float)curntTen12RMSRegs.Ten12CurrentRMSReg_N * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.pa_a   = ((float)actvPowerRegs.ActivePowerReg_A   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pa_b   = ((float)actvPowerRegs.ActivePowerReg_B   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pa_c   = ((float)actvPowerRegs.ActivePowerReg_C   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_a   = ((float)rctvPowerRegs.ReactivePowerReg_A * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_b   = ((float)rctvPowerRegs.ReactivePowerReg_B * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_c   = ((float)rctvPowerRegs.ReactivePowerReg_C * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_a = ((float)appntPowerRegs.ApparentPowerReg_A * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_b = ((float)appntPowerRegs.ApparentPowerReg_B * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_c = ((float)appntPowerRegs.ApparentPowerReg_C * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pf_a = powerFactorRegs.PowerFactorValue_A;
    r.pf_b = powerFactorRegs.PowerFactorValue_B;
    r.pf_c = powerFactorRegs.PowerFactorValue_C;
    r.freq_a = periodRegs.FrequencyValue_A;
    r.freq_b = periodRegs.FrequencyValue_B;
    r.freq_c = periodRegs.FrequencyValue_C;
    r.thd_v_a = vltgTHDRegs.VoltageTHDValue_A;
    r.thd_v_b = vltgTHDRegs.VoltageTHDValue_B;
    r.thd_v_c = vltgTHDRegs.VoltageTHDValue_C;
    r.thd_i_a = curntTHDRegs.CurrentTHDValue_A;
    r.thd_i_b = curntTHDRegs.CurrentTHDValue_B;
    r.thd_i_c = curntTHDRegs.CurrentTHDValue_C;
    r.angle_va_vb = angleRegs.AngleValue_VA_VB;
    r.angle_vb_vc = angleRegs.AngleValue_VB_VC;
    r.angle_va_vc = angleRegs.AngleValue_VA_VC;
    r.angle_va_ia = angleRegs.AngleValue_VA_IA;
    r.angle_vb_ib = angleRegs.AngleValue_VB_IB;
    r.angle_vc_ic = angleRegs.AngleValue_VC_IC;
    r.angle_ia_ib = angleRegs.AngleValue_IA_IB;
    r.angle_ib_ic = angleRegs.AngleValue_IB_IC;
    r.angle_ia_ic = angleRegs.AngleValue_IA_IC;
    r.vrms_half_a = ((float)vltgHalfRMSRegs.HalfVoltageRMSReg_A * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_half_b = ((float)vltgHalfRMSRegs.HalfVoltageRMSReg_B * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_half_c = ((float)vltgHalfRMSRegs.HalfVoltageRMSReg_C * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    if (r.irms_a < 0.05f) r.pf_a = 0.0f;
    if (r.irms_b < 0.05f) r.pf_b = 0.0f;
    if (r.irms_c < 0.05f) r.pf_c = 0.0f;
    r.vrms_fund_a = ((float)vltgFundRMSRegs.FundVoltageRMSReg_A  * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_fund_b = ((float)vltgFundRMSRegs.FundVoltageRMSReg_B  * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.vrms_fund_c = ((float)vltgFundRMSRegs.FundVoltageRMSReg_C  * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_fund_a = ((float)curntFundRMSRegs.FundCurrentRMSReg_A * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_fund_b = ((float)curntFundRMSRegs.FundCurrentRMSReg_B * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.irms_fund_c = ((float)curntFundRMSRegs.FundCurrentRMSReg_C * CAL_IRMS_CC) / SCALE_FACTOR_RMS;
    r.pa_fund_a   = ((float)actvFundPowerRegs.FundActivePowerReg_A     * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pa_fund_b   = ((float)actvFundPowerRegs.FundActivePowerReg_B     * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pa_fund_c   = ((float)actvFundPowerRegs.FundActivePowerReg_C     * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_fund_a   = ((float)rctvFundPowerRegs.FundReactivePowerReg_A   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_fund_b   = ((float)rctvFundPowerRegs.FundReactivePowerReg_B   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.pr_fund_c   = ((float)rctvFundPowerRegs.FundReactivePowerReg_C   * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_fund_a = ((float)appntFundPowerRegs.FundApparentPowerReg_A  * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_fund_b = ((float)appntFundPowerRegs.FundApparentPowerReg_B  * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    r.papp_fund_c = ((float)appntFundPowerRegs.FundApparentPowerReg_C  * CAL_POWER_CC) / SCALE_FACTOR_POWER;
    int16_t raw_temp = (int16_t)ade9000.SPI_Read_16(ADDR_TEMP_RSLT);
    r.temp_chip = (float)(raw_temp - 2624) / 53.5f;
    return r;
}


// Voltage unbalance factor u2 via symmetrical components — IEC 61000-4-30 §5.7
float calculateU2Voltage(float va, float vb, float vc,
                          float angle_ab_deg, float angle_bc_deg) {
    float theta_b = angle_ab_deg * (float)M_PI / 180.0f;
    float theta_c = (angle_ab_deg + angle_bc_deg) * (float)M_PI / 180.0f;
    float va_re = va, va_im = 0.0f;
    float vb_re = vb * cosf(theta_b), vb_im = vb * sinf(theta_b);
    float vc_re = vc * cosf(theta_c), vc_im = vc * sinf(theta_c);
    const float a_re = -0.5f, a_im = 0.8660254f;
    const float a2_re = -0.5f, a2_im = -0.8660254f;
    float aVB_re  = a_re  * vb_re - a_im  * vb_im;
    float aVB_im  = a_re  * vb_im + a_im  * vb_re;
    float a2VC_re = a2_re * vc_re - a2_im * vc_im;
    float a2VC_im = a2_re * vc_im + a2_im * vc_re;
    float v1_re   = (va_re + aVB_re + a2VC_re) / 3.0f;
    float v1_im   = (va_im + aVB_im + a2VC_im) / 3.0f;
    float v1_mag  = sqrtf(v1_re * v1_re + v1_im * v1_im);
    float a2VB_re = a2_re * vb_re - a2_im * vb_im;
    float a2VB_im = a2_re * vb_im + a2_im * vb_re;
    float aVC_re  = a_re  * vc_re - a_im  * vc_im;
    float aVC_im  = a_re  * vc_im + a_im  * vc_re;
    float v2_re   = (va_re + a2VB_re + aVC_re) / 3.0f;
    float v2_im   = (va_im + a2VB_im + aVC_im) / 3.0f;
    float v2_mag  = sqrtf(v2_re * v2_re + v2_im * v2_im);
    if (v1_mag < 0.001f) return 0.0f;
    return (v2_mag / v1_mag) * 100.0f;
}

float calculateU2Current(float ia, float ib, float ic,
                          float angle_ab_deg, float angle_bc_deg) {
    return calculateU2Voltage(ia, ib, ic, angle_ab_deg, angle_bc_deg);
}


// Event detection — called every ~8 ms from taskADE9000 sub-loop.
// Implements IEC 61000-4-30 §5.4 (SAG/SWELL) and §5.5 (interruption) logic.
void detectEvents(float v_half_a, float v_half_b, float v_half_c, uint32_t ts) {

    if (!g_measuring) {
        sag_active_a = sag_active_b = sag_active_c = false;
        swell_active_a = swell_active_b = swell_active_c = false;
        int_active_a = int_active_b = int_active_c = false;
        sag_ms_a = sag_ms_b = sag_ms_c = 0;
        swell_ms_a = swell_ms_b = swell_ms_c = 0;
        int_ms_a = int_ms_b = int_ms_c = 0;
        return;
    }

    
    if (irq1_fired) {
        irq1_fired = false;
        if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            digitalWrite(ISOLATOR_EN_PIN, HIGH);
            SPI.beginTransaction(adeSettings);
            ade9000.SPI_Write_32(ADDR_STATUS1, 0xFFFFFFFFUL);
            SPI.endTransaction();
            xSemaphoreGive(xIsolatorMutex);
        }
    }

    
    bool is_1p2w_mode = (g_system_mode == SYS_MODE_1P2W);
    bool all_below_int, any_above_int;
    if (is_1p2w_mode) {
        all_below_int = (v_half_a < INTERRUPTION_THRESHOLD);
        any_above_int = (v_half_a > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS));
    } else {
        all_below_int = (v_half_a < INTERRUPTION_THRESHOLD) &&
                        (v_half_b < INTERRUPTION_THRESHOLD) &&
                        (v_half_c < INTERRUPTION_THRESHOLD);
        any_above_int = (v_half_a > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS)) ||
                        (v_half_b > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS)) ||
                        (v_half_c > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS));
    }

    if (all_below_int) {
        if (!int_active_a) {
            
            bool any_sag = sag_active_a || sag_active_b || sag_active_c;
            if (any_sag) {
                uint8_t sp = 0; uint32_t ets = UINT32_MAX, ems = UINT32_MAX; float wm = 999.0f;
                if (sag_active_a && sag_ts_a < ets) { ets=sag_ts_a; ems=sag_ms_a; sp=0; }
                if (sag_active_b && sag_ts_b < ets) { ets=sag_ts_b; ems=sag_ms_b; sp=1; }
                if (sag_active_c && sag_ts_c < ets) { ets=sag_ts_c; ems=sag_ms_c; sp=2; }
                if (sag_active_a && sag_min_a < wm) wm = sag_min_a;
                if (sag_active_b && sag_min_b < wm) wm = sag_min_b;
                if (sag_active_c && sag_min_c < wm) wm = sag_min_c;
                if (ets != UINT32_MAX) {
                    IEC_Event se; se.type=EVT_SAG; se.phase=sp;
                    se.timestamp_start=ets; se.timestamp_end=ts;
                    se.residual_voltage=wm; se.peak_voltage=0.0f;
                    se.duration_ms=(uint32_t)(millis()-ems);
                    xQueueSend(xQueueEvents,&se,0); xQueueSend(xQueueEventComm,&se,0);
                    Serial.printf("[%s][EVT] SAG absorbido por INTERRUPCION | residual=%.3fV dur=%u ms\n",
                                  getTimeStr(), wm, se.duration_ms);
                }
            }
            sag_active_a = sag_active_b = sag_active_c = false;
            int_active_a = true;
            int_ts_a  = ts;
            int_ms_a  = millis();
            int_min_a = fminf(v_half_a, fminf(v_half_b, v_half_c));
            Serial.printf("[%s][EVT] INTERRUPCION INICIO: A=%.3fV B=%.3fV C=%.3fV\n",
                          getTimeStr(), v_half_a, v_half_b, v_half_c);
        } else {
            float cur_min = fminf(v_half_a, fminf(v_half_b, v_half_c));
            if (cur_min < int_min_a) int_min_a = cur_min;
        }
    } else if (int_active_a && any_above_int) {
        uint32_t dur = (uint32_t)(millis() - int_ms_a);
        uint8_t phase = 0;
        if      (v_half_a > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS)) phase = 0;
        else if (v_half_b > (INTERRUPTION_THRESHOLD + SAG_HYSTERESIS)) phase = 1;
        else                                                             phase = 2;
        IEC_Event evt;
        evt.type=EVT_INTERRUPTION; evt.phase=phase;
        evt.timestamp_start=int_ts_a; evt.timestamp_end=ts;
        evt.residual_voltage=int_min_a; evt.peak_voltage=0.0f;
        evt.duration_ms=dur;
        xQueueSend(xQueueEvents,&evt,0); xQueueSend(xQueueEventComm,&evt,0);
        int_active_a=false;
        g_evt_interruption=true;
        g_evt_int_phase='A'+phase; g_evt_int_ts=int_ts_a;
        g_evt_int_dur=dur; g_evt_int_res=int_min_a;
        Serial.printf("[%s][EVT] INTERRUPCION FIN | dur=%u ms residual=%.3fV\n",
                      getTimeStr(), dur, int_min_a);
    }

    
    if (!int_active_a) {

        if (v_half_a < SAG_THRESHOLD && !sag_active_a) {
            sag_active_a=true; sag_ts_a=ts; sag_ms_a=millis(); sag_min_a=v_half_a;
            Serial.printf("[%s][EVT] SAG INICIO Fase A: %.3fV\n", getTimeStr(), v_half_a);
        } else if (sag_active_a && v_half_a < sag_min_a) { sag_min_a=v_half_a; }

        
        if (!is_1p2w_mode) {
            if (v_half_b < SAG_THRESHOLD && !sag_active_b) {
                sag_active_b=true; sag_ts_b=ts; sag_ms_b=millis(); sag_min_b=v_half_b;
                Serial.printf("[%s][EVT] SAG INICIO Fase B: %.3fV\n", getTimeStr(), v_half_b);
            } else if (sag_active_b && v_half_b < sag_min_b) { sag_min_b=v_half_b; }

            if (v_half_c < SAG_THRESHOLD && !sag_active_c) {
                sag_active_c=true; sag_ts_c=ts; sag_ms_c=millis(); sag_min_c=v_half_c;
                Serial.printf("[%s][EVT] SAG INICIO Fase C: %.3fV\n", getTimeStr(), v_half_c);
            } else if (sag_active_c && v_half_c < sag_min_c) { sag_min_c=v_half_c; }
        }

        bool all_sag_done, any_sag_active;
        if (is_1p2w_mode) {
            all_sag_done   = (!sag_active_a || v_half_a > (SAG_THRESHOLD + SAG_HYSTERESIS));
            any_sag_active = sag_active_a;
        } else {
            all_sag_done   = (!sag_active_a || v_half_a > (SAG_THRESHOLD + SAG_HYSTERESIS)) &&
                             (!sag_active_b || v_half_b > (SAG_THRESHOLD + SAG_HYSTERESIS)) &&
                             (!sag_active_c || v_half_c > (SAG_THRESHOLD + SAG_HYSTERESIS));
            any_sag_active = sag_active_a || sag_active_b || sag_active_c;
        }

        if (any_sag_active && all_sag_done) {
            
            uint8_t  phase=0; uint32_t earliest_ts=UINT32_MAX, earliest_ms=UINT32_MAX; float worst_min=999.0f;
            if (sag_active_a) { if (sag_ts_a<earliest_ts){earliest_ts=sag_ts_a;earliest_ms=sag_ms_a;phase=0;} if(sag_min_a<worst_min)worst_min=sag_min_a; }
            if (!is_1p2w_mode && sag_active_b) { if (sag_ts_b<earliest_ts){earliest_ts=sag_ts_b;earliest_ms=sag_ms_b;phase=1;} if(sag_min_b<worst_min)worst_min=sag_min_b; }
            if (!is_1p2w_mode && sag_active_c) { if (sag_ts_c<earliest_ts){earliest_ts=sag_ts_c;earliest_ms=sag_ms_c;phase=2;} if(sag_min_c<worst_min)worst_min=sag_min_c; }
            if (earliest_ts==UINT32_MAX) { earliest_ts=ts; earliest_ms=millis(); }
            uint32_t dur=(uint32_t)(millis()-earliest_ms);
            IEC_Event evt;
            evt.type=EVT_SAG; evt.phase=phase;
            evt.timestamp_start=earliest_ts; evt.timestamp_end=ts;
            evt.residual_voltage=worst_min; evt.peak_voltage=0.0f;
            evt.duration_ms=dur;
            xQueueSend(xQueueEvents,&evt,0); xQueueSend(xQueueEventComm,&evt,0);
            sag_active_a=sag_active_b=sag_active_c=false;
            g_evt_sag=true;
            g_evt_sag_phase='A'+phase; g_evt_sag_ts=earliest_ts;
            g_evt_sag_dur=dur; g_evt_sag_res=worst_min;
            Serial.printf("[%s][EVT] SAG FIN | fase=%c residual=%.3fV dur=%u ms\n",
                          getTimeStr(), 'A'+phase, worst_min, dur);
        }
    }

    
    if (v_half_a > SWELL_THRESHOLD && !swell_active_a) {
        swell_active_a=true; swell_ts_a=ts; swell_ms_a=millis(); swell_max_a=v_half_a;
        Serial.printf("[%s][EVT] SWELL INICIO Fase A: %.3fV\n", getTimeStr(), v_half_a);
    } else if (swell_active_a && v_half_a > swell_max_a) { swell_max_a=v_half_a; }

    
    if (!is_1p2w_mode) {
        if (v_half_b > SWELL_THRESHOLD && !swell_active_b) {
            swell_active_b=true; swell_ts_b=ts; swell_ms_b=millis(); swell_max_b=v_half_b;
            Serial.printf("[%s][EVT] SWELL INICIO Fase B: %.3fV\n", getTimeStr(), v_half_b);
        } else if (swell_active_b && v_half_b > swell_max_b) { swell_max_b=v_half_b; }

        if (v_half_c > SWELL_THRESHOLD && !swell_active_c) {
            swell_active_c=true; swell_ts_c=ts; swell_ms_c=millis(); swell_max_c=v_half_c;
            Serial.printf("[%s][EVT] SWELL INICIO Fase C: %.3fV\n", getTimeStr(), v_half_c);
        } else if (swell_active_c && v_half_c > swell_max_c) { swell_max_c=v_half_c; }
    }

    bool all_swell_done, any_swell_active;
    if (is_1p2w_mode) {
        all_swell_done   = (!swell_active_a || v_half_a < (SWELL_THRESHOLD - SAG_HYSTERESIS));
        any_swell_active = swell_active_a;
    } else {
        all_swell_done   = (!swell_active_a || v_half_a < (SWELL_THRESHOLD - SAG_HYSTERESIS)) &&
                           (!swell_active_b || v_half_b < (SWELL_THRESHOLD - SAG_HYSTERESIS)) &&
                           (!swell_active_c || v_half_c < (SWELL_THRESHOLD - SAG_HYSTERESIS));
        any_swell_active = swell_active_a || swell_active_b || swell_active_c;
    }

    if (any_swell_active && all_swell_done) {
        
        uint8_t  phase=0; uint32_t earliest_ts=UINT32_MAX, earliest_ms=UINT32_MAX; float worst_max=0.0f;
        if (swell_active_a) { if(swell_ts_a<earliest_ts){earliest_ts=swell_ts_a;earliest_ms=swell_ms_a;phase=0;} if(swell_max_a>worst_max)worst_max=swell_max_a; }
        if (!is_1p2w_mode && swell_active_b) { if(swell_ts_b<earliest_ts){earliest_ts=swell_ts_b;earliest_ms=swell_ms_b;phase=1;} if(swell_max_b>worst_max)worst_max=swell_max_b; }
        if (!is_1p2w_mode && swell_active_c) { if(swell_ts_c<earliest_ts){earliest_ts=swell_ts_c;earliest_ms=swell_ms_c;phase=2;} if(swell_max_c>worst_max)worst_max=swell_max_c; }
        if (earliest_ts==UINT32_MAX) { earliest_ts=ts; earliest_ms=millis(); }
        uint32_t dur=(uint32_t)(millis()-earliest_ms);
        IEC_Event evt;
        evt.type=EVT_SWELL; evt.phase=phase;
        evt.timestamp_start=earliest_ts; evt.timestamp_end=ts;
        evt.residual_voltage=0.0f; evt.peak_voltage=worst_max;
        evt.duration_ms=dur;
        xQueueSend(xQueueEvents,&evt,0); xQueueSend(xQueueEventComm,&evt,0);
        swell_active_a=swell_active_b=swell_active_c=false;
        g_evt_swell=true;
        g_evt_swl_phase='A'+phase; g_evt_swl_ts=earliest_ts;
        g_evt_swl_dur=dur; g_evt_swl_peak=worst_max;
        Serial.printf("[%s][EVT] SWELL FIN | fase=%c pico=%.3fV dur=%u ms\n",
                      getTimeStr(), 'A'+phase, worst_max, dur);
    }
}


bool loadCalibrationFromEEPROM() {
    const uint16_t ACTIVE_CAL_REG_ADDRESSES[CALIBRATION_CONSTANTS_ARRAY_SIZE] = {
        ADDR_AIGAIN, ADDR_BIGAIN, ADDR_CIGAIN, ADDR_NIGAIN,
        ADDR_AVGAIN, ADDR_BVGAIN, ADDR_CVGAIN,
        ADDR_APHCAL0, ADDR_BPHCAL0, ADDR_CPHCAL0,
        ADDR_APGAIN, ADDR_BPGAIN, ADDR_CPGAIN
    };
    uint32_t calConstants[CALIBRATION_CONSTANTS_ARRAY_SIZE];
    uint32_t checksum_eeprom = 0, checksum_calculated = 0;
    Serial.println("[CAL] Leyendo EEPROM...");
    bool magic_ok = false;
    uint8_t magic = ade9000.ReadByteFromEeprom(ADDR_EEPROM_WRITTEN_BYTE);
    if (magic != EEPROM_WRITTEN) {
        Serial.printf("[CAL] Magic byte no encontrado (leido=0x%02X, esperado=0x%02X). Verificando checksum...\n",
                      magic, EEPROM_WRITTEN);
    } else {
        magic_ok = true;
    }
    for (int i = 0; i < CALIBRATION_CONSTANTS_ARRAY_SIZE; i++) {
        calConstants[i]      = ade9000.readWordFromEeprom(ADE9000_Eeprom_CalibrationRegAddress[i]);
        checksum_calculated += calConstants[i];
        delay(10);
    }
    checksum_eeprom = ade9000.readWordFromEeprom(ADDR_CHECKSUM_EEPROM);
    if (checksum_calculated != checksum_eeprom) {
        Serial.printf("[CAL] Checksum FALLIDO. Calc=0x%08X, Esperado=0x%08X\n",
                      checksum_calculated, checksum_eeprom);
        return false;
    }
    if (!magic_ok) {
        ade9000.writeByteToEeprom(ADDR_EEPROM_WRITTEN_BYTE, EEPROM_WRITTEN);
        delay(10);
        uint8_t verify = ade9000.ReadByteFromEeprom(ADDR_EEPROM_WRITTEN_BYTE);
        if (verify == EEPROM_WRITTEN) Serial.println("[CAL] Magic byte reparado y verificado.");
        else Serial.printf("[CAL] WARN: No se pudo escribir magic byte (leido=0x%02X).\n", verify);
    }
    Serial.println("[CAL] Checksum OK. Escribiendo calibracion a ADE9000...");
    SPI.beginTransaction(adeSettings);
    for (int i = 0; i < CALIBRATION_CONSTANTS_ARRAY_SIZE; i++) {
        ade9000.SPI_Write_32(ACTIVE_CAL_REG_ADDRESSES[i], calConstants[i]);
    }
    SPI.endTransaction();
    return true;
}


// Accumulates 15 x 10/12-cycle windows into one 150/180-cycle (3 s) record.
bool accumulate150(const IEC_10_12_Record& r, IEC_150_180_Record& out, uint32_t timestamp) {
    acc_150_vrms_a  += r.vrms_a  * r.vrms_a;  acc_150_vrms_b  += r.vrms_b  * r.vrms_b;  acc_150_vrms_c  += r.vrms_c  * r.vrms_c;
    acc_150_irms_a  += r.irms_a  * r.irms_a;  acc_150_irms_b  += r.irms_b  * r.irms_b;  acc_150_irms_c  += r.irms_c  * r.irms_c;
    acc_150_irms_n  += r.irms_n  * r.irms_n;
    acc_150_pa_a    += r.pa_a;   acc_150_pa_b    += r.pa_b;   acc_150_pa_c    += r.pa_c;
    acc_150_pr_a    += r.pr_a;   acc_150_pr_b    += r.pr_b;   acc_150_pr_c    += r.pr_c;
    acc_150_papp_a  += r.papp_a; acc_150_papp_b  += r.papp_b; acc_150_papp_c  += r.papp_c;
    acc_150_pf_a    += r.pf_a;   acc_150_pf_b    += r.pf_b;   acc_150_pf_c    += r.pf_c;
    acc_150_freq    += r.freq_a; acc_150_freq_b  += r.freq_b; acc_150_freq_c  += r.freq_c;
    acc_150_thd_v_a += r.thd_v_a; acc_150_thd_v_b += r.thd_v_b; acc_150_thd_v_c += r.thd_v_c;
    acc_150_thd_i_a += r.thd_i_a; acc_150_thd_i_b += r.thd_i_b; acc_150_thd_i_c += r.thd_i_c;
    acc_150_pa_fund_a   += r.pa_fund_a;   acc_150_pa_fund_b   += r.pa_fund_b;   acc_150_pa_fund_c   += r.pa_fund_c;
    acc_150_pr_fund_a   += r.pr_fund_a;   acc_150_pr_fund_b   += r.pr_fund_b;   acc_150_pr_fund_c   += r.pr_fund_c;
    acc_150_papp_fund_a += r.papp_fund_a; acc_150_papp_fund_b += r.papp_fund_b; acc_150_papp_fund_c += r.papp_fund_c;
    acc_150_vrms_fund_a += r.vrms_fund_a * r.vrms_fund_a; acc_150_vrms_fund_b += r.vrms_fund_b * r.vrms_fund_b; acc_150_vrms_fund_c += r.vrms_fund_c * r.vrms_fund_c;
    acc_150_irms_fund_a += r.irms_fund_a * r.irms_fund_a; acc_150_irms_fund_b += r.irms_fund_b * r.irms_fund_b; acc_150_irms_fund_c += r.irms_fund_c * r.irms_fund_c;
    acc_150_temp        += r.temp_chip;
    if (r.event_flag) flag_150 = true;
    cnt_150++;
    if (cnt_150 < WINDOWS_PER_150_180) return false;
    float n = (float)cnt_150;
    out.vrms_a  = sqrtf(acc_150_vrms_a/n);  out.vrms_b  = sqrtf(acc_150_vrms_b/n);  out.vrms_c  = sqrtf(acc_150_vrms_c/n);
    out.irms_a  = sqrtf(acc_150_irms_a/n);  out.irms_b  = sqrtf(acc_150_irms_b/n);  out.irms_c  = sqrtf(acc_150_irms_c/n);
    out.irms_n  = sqrtf(acc_150_irms_n/n);
    out.pa_a    = acc_150_pa_a/n;   out.pa_b    = acc_150_pa_b/n;   out.pa_c    = acc_150_pa_c/n;
    out.pr_a    = acc_150_pr_a/n;   out.pr_b    = acc_150_pr_b/n;   out.pr_c    = acc_150_pr_c/n;
    out.papp_a  = acc_150_papp_a/n; out.papp_b  = acc_150_papp_b/n; out.papp_c  = acc_150_papp_c/n;
    out.pf_a    = acc_150_pf_a/n;   out.pf_b    = acc_150_pf_b/n;   out.pf_c    = acc_150_pf_c/n;
    out.freq_a  = acc_150_freq/n;   out.freq_b  = acc_150_freq_b/n; out.freq_c  = acc_150_freq_c/n;
    out.thd_v_a = acc_150_thd_v_a/n; out.thd_v_b = acc_150_thd_v_b/n; out.thd_v_c = acc_150_thd_v_c/n;
    out.thd_i_a = acc_150_thd_i_a/n; out.thd_i_b = acc_150_thd_i_b/n; out.thd_i_c = acc_150_thd_i_c/n;
    out.pa_fund_a   = acc_150_pa_fund_a/n;   out.pa_fund_b   = acc_150_pa_fund_b/n;   out.pa_fund_c   = acc_150_pa_fund_c/n;
    out.pr_fund_a   = acc_150_pr_fund_a/n;   out.pr_fund_b   = acc_150_pr_fund_b/n;   out.pr_fund_c   = acc_150_pr_fund_c/n;
    out.papp_fund_a = acc_150_papp_fund_a/n; out.papp_fund_b = acc_150_papp_fund_b/n; out.papp_fund_c = acc_150_papp_fund_c/n;
    out.vrms_fund_a = sqrtf(acc_150_vrms_fund_a/n); out.vrms_fund_b = sqrtf(acc_150_vrms_fund_b/n); out.vrms_fund_c = sqrtf(acc_150_vrms_fund_c/n);
    out.irms_fund_a = sqrtf(acc_150_irms_fund_a/n); out.irms_fund_b = sqrtf(acc_150_irms_fund_b/n); out.irms_fund_c = sqrtf(acc_150_irms_fund_c/n);
    out.temp_chip   = acc_150_temp/n;
    out.u2_voltage  = calculateU2Voltage(out.vrms_a, out.vrms_b, out.vrms_c, r.angle_va_vb, r.angle_vb_vc);
    out.u2_current  = calculateU2Current(out.irms_a, out.irms_b, out.irms_c, angleRegs.AngleValue_IA_IB, angleRegs.AngleValue_IB_IC);
    out.timestamp    = timestamp;
    out.event_flag   = flag_150;
    out.window_count = cnt_150;
    out.angle_va_vb = r.angle_va_vb; out.angle_vb_vc = r.angle_vb_vc; out.angle_va_vc = r.angle_va_vc;
    out.angle_va_ia = r.angle_va_ia; out.angle_vb_ib = r.angle_vb_ib; out.angle_vc_ic = r.angle_vc_ic;
    out.angle_ia_ib = r.angle_ia_ib; out.angle_ib_ic = r.angle_ib_ic; out.angle_ia_ic = r.angle_ia_ic;
    out.vrms_half_a = r.vrms_half_a; out.vrms_half_b = r.vrms_half_b; out.vrms_half_c = r.vrms_half_c;
    acc_150_vrms_a = acc_150_vrms_b = acc_150_vrms_c = 0;
    acc_150_irms_a = acc_150_irms_b = acc_150_irms_c = acc_150_irms_n = 0;
    acc_150_pa_a = acc_150_pa_b = acc_150_pa_c = 0; acc_150_pr_a = acc_150_pr_b = acc_150_pr_c = 0;
    acc_150_papp_a = acc_150_papp_b = acc_150_papp_c = 0; acc_150_pf_a = acc_150_pf_b = acc_150_pf_c = 0;
    acc_150_freq = acc_150_freq_b = acc_150_freq_c = 0;
    acc_150_thd_v_a = acc_150_thd_v_b = acc_150_thd_v_c = 0; acc_150_thd_i_a = acc_150_thd_i_b = acc_150_thd_i_c = 0;
    acc_150_pa_fund_a = acc_150_pa_fund_b = acc_150_pa_fund_c = 0;
    acc_150_pr_fund_a = acc_150_pr_fund_b = acc_150_pr_fund_c = 0;
    acc_150_papp_fund_a = acc_150_papp_fund_b = acc_150_papp_fund_c = 0;
    acc_150_vrms_fund_a = acc_150_vrms_fund_b = acc_150_vrms_fund_c = 0;
    acc_150_irms_fund_a = acc_150_irms_fund_b = acc_150_irms_fund_c = 0;
    acc_150_temp = 0; cnt_150 = 0; flag_150 = false;
    return true;
}


// Accumulates 200 x 3-second records into one 10-minute record.
bool accumulate10min(const IEC_150_180_Record& r, IEC_10MIN_Record& out, uint32_t timestamp) {
    if (cnt_10m == 0) ts_10m_start = (uint32_t)(r.timestamp / 1000ULL);  
    acc_10m_vrms_a  += r.vrms_a*r.vrms_a;   acc_10m_vrms_b  += r.vrms_b*r.vrms_b;   acc_10m_vrms_c  += r.vrms_c*r.vrms_c;
    acc_10m_irms_a  += r.irms_a*r.irms_a;   acc_10m_irms_b  += r.irms_b*r.irms_b;   acc_10m_irms_c  += r.irms_c*r.irms_c;
    acc_10m_irms_n  += r.irms_n*r.irms_n;
    acc_10m_pa_a    += r.pa_a;   acc_10m_pa_b    += r.pa_b;   acc_10m_pa_c    += r.pa_c;
    acc_10m_pr_a    += r.pr_a;   acc_10m_pr_b    += r.pr_b;   acc_10m_pr_c    += r.pr_c;
    acc_10m_papp_a  += r.papp_a; acc_10m_papp_b  += r.papp_b; acc_10m_papp_c  += r.papp_c;
    acc_10m_pf_a    += r.pf_a;   acc_10m_pf_b    += r.pf_b;   acc_10m_pf_c    += r.pf_c;
    acc_10m_freq    += r.freq_a;
    acc_10m_thd_v_a += r.thd_v_a; acc_10m_thd_v_b += r.thd_v_b; acc_10m_thd_v_c += r.thd_v_c;
    acc_10m_thd_i_a += r.thd_i_a; acc_10m_thd_i_b += r.thd_i_b; acc_10m_thd_i_c += r.thd_i_c;
    acc_10m_pa_fund_a   += r.pa_fund_a;   acc_10m_pa_fund_b   += r.pa_fund_b;   acc_10m_pa_fund_c   += r.pa_fund_c;
    acc_10m_pr_fund_a   += r.pr_fund_a;   acc_10m_pr_fund_b   += r.pr_fund_b;   acc_10m_pr_fund_c   += r.pr_fund_c;
    acc_10m_papp_fund_a += r.papp_fund_a; acc_10m_papp_fund_b += r.papp_fund_b; acc_10m_papp_fund_c += r.papp_fund_c;
    acc_10m_vrms_fund_a += r.vrms_fund_a*r.vrms_fund_a; acc_10m_vrms_fund_b += r.vrms_fund_b*r.vrms_fund_b; acc_10m_vrms_fund_c += r.vrms_fund_c*r.vrms_fund_c;
    acc_10m_irms_fund_a += r.irms_fund_a*r.irms_fund_a; acc_10m_irms_fund_b += r.irms_fund_b*r.irms_fund_b; acc_10m_irms_fund_c += r.irms_fund_c*r.irms_fund_c;
    acc_10m_temp += r.temp_chip; acc_10m_u2v += r.u2_voltage; acc_10m_u2i += r.u2_current;
    if (r.event_flag) flag_10m = true;
    cnt_10m++;
    if (cnt_10m < WINDOWS_PER_10MIN) return false;
    float n = (float)cnt_10m;
    out.vrms_a  = sqrtf(acc_10m_vrms_a/n);  out.vrms_b  = sqrtf(acc_10m_vrms_b/n);  out.vrms_c  = sqrtf(acc_10m_vrms_c/n);
    out.irms_a  = sqrtf(acc_10m_irms_a/n);  out.irms_b  = sqrtf(acc_10m_irms_b/n);  out.irms_c  = sqrtf(acc_10m_irms_c/n);
    out.irms_n  = sqrtf(acc_10m_irms_n/n);
    out.pa_a    = acc_10m_pa_a/n;   out.pa_b    = acc_10m_pa_b/n;   out.pa_c    = acc_10m_pa_c/n;
    out.pr_a    = acc_10m_pr_a/n;   out.pr_b    = acc_10m_pr_b/n;   out.pr_c    = acc_10m_pr_c/n;
    out.papp_a  = acc_10m_papp_a/n; out.papp_b  = acc_10m_papp_b/n; out.papp_c  = acc_10m_papp_c/n;
    out.pf_a    = acc_10m_pf_a/n;   out.pf_b    = acc_10m_pf_b/n;   out.pf_c    = acc_10m_pf_c/n;
    out.freq_a  = acc_10m_freq/n;
    out.thd_v_a = acc_10m_thd_v_a/n; out.thd_v_b = acc_10m_thd_v_b/n; out.thd_v_c = acc_10m_thd_v_c/n;
    out.thd_i_a = acc_10m_thd_i_a/n; out.thd_i_b = acc_10m_thd_i_b/n; out.thd_i_c = acc_10m_thd_i_c/n;
    out.pa_fund_a   = acc_10m_pa_fund_a/n;   out.pa_fund_b   = acc_10m_pa_fund_b/n;   out.pa_fund_c   = acc_10m_pa_fund_c/n;
    out.pr_fund_a   = acc_10m_pr_fund_a/n;   out.pr_fund_b   = acc_10m_pr_fund_b/n;   out.pr_fund_c   = acc_10m_pr_fund_c/n;
    out.papp_fund_a = acc_10m_papp_fund_a/n; out.papp_fund_b = acc_10m_papp_fund_b/n; out.papp_fund_c = acc_10m_papp_fund_c/n;
    out.vrms_fund_a = sqrtf(acc_10m_vrms_fund_a/n); out.vrms_fund_b = sqrtf(acc_10m_vrms_fund_b/n); out.vrms_fund_c = sqrtf(acc_10m_vrms_fund_c/n);
    out.irms_fund_a = sqrtf(acc_10m_irms_fund_a/n); out.irms_fund_b = sqrtf(acc_10m_irms_fund_b/n); out.irms_fund_c = sqrtf(acc_10m_irms_fund_c/n);
    out.temp_chip = acc_10m_temp/n; out.u2_voltage = acc_10m_u2v/n; out.u2_current = acc_10m_u2i/n;
    out.timestamp_start = ts_10m_start; out.timestamp_end = timestamp;
    out.event_flag = flag_10m; out.window_count = cnt_10m;
    acc_10m_vrms_a = acc_10m_vrms_b = acc_10m_vrms_c = 0; acc_10m_irms_a = acc_10m_irms_b = acc_10m_irms_c = acc_10m_irms_n = 0;
    acc_10m_pa_a = acc_10m_pa_b = acc_10m_pa_c = 0; acc_10m_pr_a = acc_10m_pr_b = acc_10m_pr_c = 0;
    acc_10m_papp_a = acc_10m_papp_b = acc_10m_papp_c = 0; acc_10m_pf_a = acc_10m_pf_b = acc_10m_pf_c = 0;
    acc_10m_freq = 0; acc_10m_thd_v_a = acc_10m_thd_v_b = acc_10m_thd_v_c = 0; acc_10m_thd_i_a = acc_10m_thd_i_b = acc_10m_thd_i_c = 0;
    acc_10m_pa_fund_a = acc_10m_pa_fund_b = acc_10m_pa_fund_c = 0;
    acc_10m_pr_fund_a = acc_10m_pr_fund_b = acc_10m_pr_fund_c = 0;
    acc_10m_papp_fund_a = acc_10m_papp_fund_b = acc_10m_papp_fund_c = 0;
    acc_10m_vrms_fund_a = acc_10m_vrms_fund_b = acc_10m_vrms_fund_c = 0;
    acc_10m_irms_fund_a = acc_10m_irms_fund_b = acc_10m_irms_fund_c = 0;
    acc_10m_temp = 0; acc_10m_u2v = acc_10m_u2i = 0; cnt_10m = 0; flag_10m = false;
    return true;
}


// Core 0 task: continuous IEC measurement pipeline + 8 ms event detection sub-loop.
void taskADE9000(void* pvParameters) {
    Serial.println("[CORE0] taskADE9000 v27.5 iniciado — sub-loop 8ms para deteccion de eventos.");
    uint32_t irq0_count    = 0;
    uint32_t agg_150_count = 0;
    uint32_t agg_10m_count = 0;

    
    const TickType_t HALF_CYCLE_TICKS   = pdMS_TO_TICKS(8);
    const uint8_t    SUBCYCLES_PER_WIN  = 25;
    uint8_t subcycle_count = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, HALF_CYCLE_TICKS);
        subcycle_count++;

        
        if (!g_measuring) {
            subcycle_count = 0;
            cnt_150 = 0; cnt_10m = 0; flag_150 = false;
            acc_150_vrms_a = acc_150_vrms_b = acc_150_vrms_c = 0;
            acc_150_irms_a = acc_150_irms_b = acc_150_irms_c = acc_150_irms_n = 0;
            acc_10m_vrms_a = acc_10m_vrms_b = acc_10m_vrms_c = 0;
            acc_10m_irms_a = acc_10m_irms_b = acc_10m_irms_c = acc_10m_irms_n = 0;
            continue;
        }

        if (xSemaphoreTake(xIsolatorMutex, 0) == pdTRUE) {
            
            
            struct HalfVoltageRMSRegs localHalfRegs;
            digitalWrite(ISOLATOR_EN_PIN, HIGH);
            SPI.beginTransaction(adeSettings);
            ade9000.ReadHalfVoltageRMSRegs(&localHalfRegs);
            SPI.endTransaction();
            float va = ((float)localHalfRegs.HalfVoltageRMSReg_A * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
            float vb = ((float)localHalfRegs.HalfVoltageRMSReg_B * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
            float vc = ((float)localHalfRegs.HalfVoltageRMSReg_C * CAL_VRMS_CC) / SCALE_FACTOR_RMS;
            xSemaphoreGive(xIsolatorMutex);
            detectEvents(va, vb, vc, getUnixTS());
        }
        

        if (subcycle_count < SUBCYCLES_PER_WIN) continue;
        subcycle_count = 0;
        irq0_count++;

        
        {
            uint32_t t0 = millis();
            bool irq0_ok = false;
            while ((millis() - t0) < 50) {
                if (irq0_rms_ready) { irq0_rms_ready = false; irq0_ok = true; break; }
                vTaskDelay(1);
            }
            if (!irq0_ok) {
                irq0_rms_ready = false;
                Serial.printf("[%s][CORE0] WARN: timeout IRQ0 ventana %u\n", getTimeStr(), irq0_count);
            }
        }

        uint32_t ts = getUnixTS();  

        if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(180)) == pdTRUE) {
            digitalWrite(ISOLATOR_EN_PIN, HIGH);
            SPI.beginTransaction(adeSettings);
            readAllADERegisters();
            ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFFUL);
            SPI.endTransaction();
            xSemaphoreGive(xIsolatorMutex);
        } else {
            Serial.printf("[%s][CORE0] WARN: timeout mutex, lectura omitida\n", getTimeStr());
            continue;
        }

        IEC_10_12_Record rec = convertToPhysical(ts);

        if (sag_active_a || sag_active_b || sag_active_c ||
            swell_active_a || swell_active_b || swell_active_c ||
            int_active_a || int_active_b || int_active_c) {
            rec.event_flag = true;
        }

        
        IEC_150_180_Record rec150;
        if (accumulate150(rec, rec150, ts)) {
            
            
            rec150.timestamp = getRecordTs();
            agg_150_count++;
            Serial.printf("[%s][IEC] Ventana 150-ciclos #%u completa. VA=%.3fV VB=%.3fV VC=%.3fV u2=%.2f%%\n",
                          getTimeStr(), agg_150_count,
                          rec150.vrms_a, rec150.vrms_b, rec150.vrms_c, rec150.u2_voltage);

            if (xQueueSend(xQueue150Comm, &rec150, 0) != pdTRUE) {
                Serial.printf("[%s][CORE0] WARN: xQueue150Comm full — saving to PENDING\n", getTimeStr());
                pendingEnqueue(rec150);
            }

            IEC_10MIN_Record rec10m;
            if (accumulate10min(rec150, rec10m, ts)) {
                agg_10m_count++;
                Serial.printf("[%s][IEC] *** VENTANA 10-MIN #%u COMPLETA ***\n", getTimeStr(), agg_10m_count);
                Serial.printf("[%s][IEC] VA=%.3fV VB=%.3fV VC=%.3fV | IA=%.3fA IB=%.3fA IC=%.3fA\n",
                              getTimeStr(), rec10m.vrms_a, rec10m.vrms_b, rec10m.vrms_c,
                              rec10m.irms_a, rec10m.irms_b, rec10m.irms_c);
                Serial.printf("[%s][IEC] Freq=%.3fHz u2v=%.2f%% u2i=%.2f%% Eventos=%d Ventanas=%d\n",
                              getTimeStr(), rec10m.freq_a, rec10m.u2_voltage,
                              rec10m.u2_current, rec10m.event_flag, rec10m.window_count);
                if (xQueueSend(xQueue10min, &rec10m, pdMS_TO_TICKS(50)) != pdTRUE) {
                    Serial.printf("[%s][CORE0] WARN: cola 10min llena, registro descartado\n", getTimeStr());
                }
            }
        }

        if (irq0_count % 15 == 0) {
            Serial.printf("[%s][CORE0] Ventana=%u | VA=%.3fV IA=%.3fA | Freq=%.3fHz | THDv_A=%.2f%%\n",
                          getTimeStr(), irq0_count, rec.vrms_a, rec.irms_a, rec.freq_a, rec.thd_v_a);
        }
    }
}


// Core 1 task: writes 10-minute aggregates and events to SD card.
void taskSD(void* pvParameters) {
    Serial.println("[CORE1] taskSD iniciado en Core 1.");
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (true) {
        IEC_10MIN_Record rec10m;
        if (xQueueReceive(xQueue10min, &rec10m, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("[%s][SD] Registro 10-min recibido. Escribiendo a SD...\n", getTimeStr());
            if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                digitalWrite(ISOLATOR_EN_PIN, LOW);
                delayMicroseconds(100);
                char filename[32];
                DateTime now = rtc.now();
                sprintf(filename, "/PQM_%04d%02d%02d.csv", now.year(), now.month(), now.day());
                File f = SD.open(filename, FILE_APPEND);
                if (!f) {
                    Serial.printf("[%s][SD] ERROR: no se pudo abrir CSV\n", getTimeStr());
                } else {
                    if (f.size() == 0) {
                        f.println("ts_start,ts_end,event,"
                                  "VA,VB,VC,IA,IB,IC,IN,"
                                  "PA_A,PA_B,PA_C,PR_A,PR_B,PR_C,PAPP_A,PAPP_B,PAPP_C,"
                                  "PA_FUND_A,PA_FUND_B,PA_FUND_C,"
                                  "PR_FUND_A,PR_FUND_B,PR_FUND_C,"
                                  "PAPP_FUND_A,PAPP_FUND_B,PAPP_FUND_C,"
                                  "VFUND_A,VFUND_B,VFUND_C,"
                                  "IFUND_A,IFUND_B,IFUND_C,"
                                  "PF_A,PF_B,PF_C,FREQ,"
                                  "VTHD_A,VTHD_B,VTHD_C,ITHD_A,ITHD_B,ITHD_C,"
                                  "U2V,U2I,TEMP_CHIP,BATT_V,BATT_PCT,BATT_SRC,WINDOWS");
                    }
                    char line[768];
                    snprintf(line, sizeof(line),
                             "%lu,%lu,%d,"
                             "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                             "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                             "%.4f,%.4f,%.4f,%.4f,"
                             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                             "%.3f,%.3f,%.2f,%.2f,%u,%s,%d",
                             rec10m.timestamp_start, rec10m.timestamp_end, (int)rec10m.event_flag,
                             rec10m.vrms_a, rec10m.vrms_b, rec10m.vrms_c,
                             rec10m.irms_a, rec10m.irms_b, rec10m.irms_c, rec10m.irms_n,
                             rec10m.pa_a, rec10m.pa_b, rec10m.pa_c,
                             rec10m.pr_a, rec10m.pr_b, rec10m.pr_c,
                             rec10m.papp_a, rec10m.papp_b, rec10m.papp_c,
                             rec10m.pa_fund_a, rec10m.pa_fund_b, rec10m.pa_fund_c,
                             rec10m.pr_fund_a, rec10m.pr_fund_b, rec10m.pr_fund_c,
                             rec10m.papp_fund_a, rec10m.papp_fund_b, rec10m.papp_fund_c,
                             rec10m.vrms_fund_a, rec10m.vrms_fund_b, rec10m.vrms_fund_c,
                             rec10m.irms_fund_a, rec10m.irms_fund_b, rec10m.irms_fund_c,
                             rec10m.pf_a, rec10m.pf_b, rec10m.pf_c, rec10m.freq_a,
                             rec10m.thd_v_a, rec10m.thd_v_b, rec10m.thd_v_c,
                             rec10m.thd_i_a, rec10m.thd_i_b, rec10m.thd_i_c,
                             rec10m.u2_voltage, rec10m.u2_current, rec10m.temp_chip,
                             g_batt_voltage, g_batt_percent,
                             g_batt_on_battery ? "BAT" : "AC",
                             rec10m.window_count);
                    if (f.println(line)) { f.flush(); Serial.printf("[%s][SD] Registro 10-min guardado.\n", getTimeStr()); }
                    else Serial.printf("[%s][SD] ERROR: fallo al escribir linea CSV.\n", getTimeStr());
                    f.close();
                }
                delayMicroseconds(100);
                digitalWrite(ISOLATOR_EN_PIN, HIGH);
                xSemaphoreGive(xIsolatorMutex);
            }
        }

        IEC_Event evt;
        while (xQueueReceive(xQueueEvents, &evt, 0) == pdTRUE) {
            if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                digitalWrite(ISOLATOR_EN_PIN, LOW);
                delayMicroseconds(100);
                File fe = SD.open("/EVENTS.csv", FILE_APPEND);
                if (fe) {
                    if (fe.size() == 0) fe.println("type,phase,ts_start,ts_end,residual_v,peak_v,duration_ms");
                    const char* type_str = (evt.type == EVT_SAG)          ? "SAG"          :
                                           (evt.type == EVT_SWELL)        ? "SWELL"        :
                                           (evt.type == EVT_INTERRUPTION) ? "INTERRUPTION" : "UNKNOWN";
                    char evtline[160];
                    snprintf(evtline, sizeof(evtline), "%s,%d,%lu,%lu,%.4f,%.4f,%u",
                             type_str, evt.phase,
                             evt.timestamp_start, evt.timestamp_end,
                             evt.residual_voltage, evt.peak_voltage,
                             evt.duration_ms);
                    fe.println(evtline); fe.flush(); fe.close();
                    Serial.printf("[%s][SD] Evento %s Fase %c guardado.\n",
                                  getTimeStr(), type_str, 'A' + evt.phase);
                }
                delayMicroseconds(100);
                digitalWrite(ISOLATOR_EN_PIN, HIGH);
                xSemaphoreGive(xIsolatorMutex);
            }
        }
        taskYIELD();
    }
}


static void hmi_write_u2() { pcfU2.write8(u2_state); }
static void hmi_write_u3() { pcfU3.write8(u3_state | 0x0F); }
static void hmi_beep_short() { digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(50)); digitalWrite(BUZZER_PIN, LOW); }
static void hmi_beep_start() {
    for (int i = 0; i < 3; i++) { digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(80)); digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(80)); }
}
static void hmi_beep_stop() { digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(300)); digitalWrite(BUZZER_PIN, LOW); }

static void hmi_apply_system_mode(uint8_t mode) {
    uint16_t accmode_val = (mode == SYS_MODE_3P3W) ? ACCMODE_3P3W : ACCMODE_3P4W;
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        digitalWrite(ISOLATOR_EN_PIN, HIGH);
        SPI.beginTransaction(adeSettings);
        ade9000.SPI_Write_16(ADDR_ACCMODE, accmode_val);
        SPI.endTransaction();
        xSemaphoreGive(xIsolatorMutex);
        
        g_u_din = (mode == SYS_MODE_3P3W) ? U_DIN_3P3W : U_DIN_1P2W_3P4W;
        Serial.printf("[HMI] ACCMODE=0x%04X aplicado (%s) | U_DIN=%.1fV\n", accmode_val,
                      mode == SYS_MODE_1P2W ? "1P2W" : (mode == SYS_MODE_3P4W ? "3P4W" : "3P3W"),
                      g_u_din);
        
        setupHardwareEventThresholds();
    }
}


// Core 1 task: button debounce, LED control, buzzer, relay, START/STOP logic.
void taskHMI(void* pvParameters) {
    Serial.println("[CORE1] taskHMI iniciado.");
    uint8_t  comm_blinks_left = 0;
    unsigned long t_comm_blink = 0;
    uint8_t  beep_state = 0, beep_type = BEEP_NONE, beep_count = 0;
    unsigned long t_beep = 0;
    bool    led_err_last  = false;
    uint8_t led_mode_last = 0;

    while (true) {
        unsigned long now = millis();

        if (g_button_pressed) {
            vTaskDelay(pdMS_TO_TICKS(50));
            uint8_t pins = pcfU3.read8();
            g_button_pressed = false;

            if (bitRead(pins, BTN_START_STOP)) {
                if (!g_measuring) {
                    bool sd_ok = false;
                    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        digitalWrite(ISOLATOR_EN_PIN, LOW);
                        delayMicroseconds(500);  
                        File testFile = SD.open("/sdtest.tmp", FILE_WRITE);
                        if (testFile) { testFile.println("ok"); testFile.close(); SD.remove("/sdtest.tmp"); sd_ok = true; }
                        else {
                            SD.end(); delay(50);
                            if (SD.begin(SD_CS_PIN)) {
                                File tf2 = SD.open("/sdtest.tmp", FILE_WRITE);
                                if (tf2) { tf2.println("ok"); tf2.close(); SD.remove("/sdtest.tmp"); sd_ok = true; }
                            }
                        }
                        digitalWrite(ISOLATOR_EN_PIN, HIGH);
                        xSemaphoreGive(xIsolatorMutex);
                    }
                    if (!sd_ok) {
                        Serial.println("[HMI] WARN — SD no disponible.");
                        if (g_comm_mode == 2) {
                            Serial.println("[HMI] Modo SD activo — bloqueando START hasta insertar SD.");
                            for (int i = 0; i < 3; i++) { digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(100)); digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(100)); }
                            g_led_error_sd = true;
                            bitWrite(u3_state, LED_ERROR, LED_ON);
                            hmi_write_u3();
                            vTaskDelay(pdMS_TO_TICKS(20));
                            continue;
                        } else {
                            for (int i = 0; i < 2; i++) { digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(100)); digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(100)); }
                        }
                    } else {
                        if (bitRead(u3_state, LED_ERROR) == LED_ON) {
                            g_led_error_sd = false;
                            bitWrite(u3_state, LED_ERROR, LED_OFF);
                            hmi_write_u3();
                        }
                    }
                    g_sd_error = !sd_ok;

                    
                    g_vpeak = 0.0f; g_ipeak = 0.0f;
                    g_oc_flag = 0; g_oc_phase = 0;
                    g_oc_peak_a = g_oc_peak_b = g_oc_peak_c = 0.0f;
                    g_phase_seq = 0;

                    
                    cnt_10m = 0; flag_10m = false; ts_10m_start = 0;
                    acc_10m_vrms_a = acc_10m_vrms_b = acc_10m_vrms_c = 0;
                    acc_10m_irms_a = acc_10m_irms_b = acc_10m_irms_c = acc_10m_irms_n = 0;
                    acc_10m_pa_a = acc_10m_pa_b = acc_10m_pa_c = 0;
                    acc_10m_pr_a = acc_10m_pr_b = acc_10m_pr_c = 0;
                    acc_10m_papp_a = acc_10m_papp_b = acc_10m_papp_c = 0;
                    acc_10m_pf_a = acc_10m_pf_b = acc_10m_pf_c = 0;
                    acc_10m_freq = 0;
                    acc_10m_thd_v_a = acc_10m_thd_v_b = acc_10m_thd_v_c = 0;
                    acc_10m_thd_i_a = acc_10m_thd_i_b = acc_10m_thd_i_c = 0;
                    acc_10m_pa_fund_a = acc_10m_pa_fund_b = acc_10m_pa_fund_c = 0;
                    acc_10m_pr_fund_a = acc_10m_pr_fund_b = acc_10m_pr_fund_c = 0;
                    acc_10m_papp_fund_a = acc_10m_papp_fund_b = acc_10m_papp_fund_c = 0;
                    acc_10m_vrms_fund_a = acc_10m_vrms_fund_b = acc_10m_vrms_fund_c = 0;
                    acc_10m_irms_fund_a = acc_10m_irms_fund_b = acc_10m_irms_fund_c = 0;
                    acc_10m_temp = 0; acc_10m_u2v = acc_10m_u2i = 0;

                    
                    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        ade9000.SPI_Write_32(ADDR_VPEAK, 0xFFFFFFFF);
                        ade9000.SPI_Write_32(ADDR_IPEAK, 0xFFFFFFFF);
                        xSemaphoreGive(xIsolatorMutex);
                    }

                    g_gps_fix = 0; g_gps_lat = 0.0f; g_gps_lon = 0.0f; g_gps_hdop = 99.9f;
                    g_gps_last_ms = millis();

                    
                    sag_active_a = sag_active_b = sag_active_c = false;
                    swell_active_a = swell_active_b = swell_active_c = false;
                    int_active_a = int_active_b = int_active_c = false;
                    sag_min_a = sag_min_b = sag_min_c = 999.0f;
                    int_min_a = int_min_b = int_min_c = 999.0f;
                    swell_max_a = swell_max_b = swell_max_c = 0.0f;

                    
                    sag_ts_a = sag_ts_b = sag_ts_c = 0;
                    sag_ms_a = sag_ms_b = sag_ms_c = 0;
                    swell_ts_a = swell_ts_b = swell_ts_c = 0;
                    swell_ms_a = swell_ms_b = swell_ms_c = 0;
                    setupHardwareEventThresholds();
                    Serial.println("[HMI] START — Umbrales hardware DIP/SWELL reconfigurados.");

                    
                    g_evt_sag          = false;
                    g_evt_swell        = false;
                    g_evt_interruption = false;
                    g_evt_sag_phase='-'; g_evt_sag_ts=0; g_evt_sag_dur=0; g_evt_sag_res=0.0f;
                    g_evt_int_phase='-'; g_evt_int_ts=0; g_evt_int_dur=0; g_evt_int_res=0.0f;
                    g_evt_swl_phase='-'; g_evt_swl_ts=0; g_evt_swl_dur=0; g_evt_swl_peak=0.0f;

                    
                    g_alarm_batt_sent     = false;
                    g_alarm_temp_sent     = false;
                    g_alarm_failover_sent = false;

                    g_start_attrs_sent = false;

                    if (g_last_ntp_sync == 0) {
                        Serial.println("[HMI] WARN — RTC no sincronizado con NTP. Timestamps pueden ser invalidos.");
                    }

                    
                    if (g_comm_mode == 2) {
                        
                        if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                            digitalWrite(ISOLATOR_EN_PIN, LOW);
                            delayMicroseconds(500);
                            SD.remove(PENDING_FILE);
                            digitalWrite(ISOLATOR_EN_PIN, HIGH);
                            xSemaphoreGive(xIsolatorMutex);
                            Serial.println("[HMI] START — PENDING.csv borrado (modo SD, sin nube).");
                        }
                    } else {
                        
                        g_pending_flush_on_start = true;
                        Serial.println("[HMI] START — PENDING flush solicitado a taskComm.");
                    }

                    hmi_beep_start();
                    
                    
                    if (bitRead(u2_state, LED_LOCAL_SETUP) == LED_ON) {
                        Serial.println("[HMI] START — Modo LOCAL SETUP: relay permanece ON (red principal).");
                    } else {
                        digitalWrite(RELAY_PIN, RELAY_OFF);
                    }
                    bitWrite(u3_state, LED_STATUS,      LED_ON);
                    bitWrite(u2_state, LED_LOCAL_SETUP, LED_OFF);
                    hmi_write_u2(); hmi_write_u3();
                    
                    
                    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        digitalWrite(ISOLATOR_EN_PIN, HIGH);
                        SPI.beginTransaction(adeSettings);
                        uint16_t regs_hi[] = {0x2E7,0x323,0x35F,0x2F1,0x32D,0x369,0x2FB,0x337,0x373,
                                              0x305,0x341,0x37D,0x30F,0x34B,0x387,0x319,0x355,0x391};
                        uint16_t regs_lo[] = {0x2E6,0x322,0x35E,0x2F0,0x32C,0x368,0x2FA,0x336,0x372,
                                              0x304,0x340,0x37C,0x30E,0x34A,0x386,0x318,0x354,0x390};
                        for (int i = 0; i < 18; i++) {
                            ade9000.SPI_Write_32(regs_hi[i], 0x00000000UL);
                            ade9000.SPI_Write_32(regs_lo[i], 0x00000000UL);
                        }
                        SPI.endTransaction();
                        xSemaphoreGive(xIsolatorMutex);
                        Serial.println("[HMI] START — Acumuladores de energia reseteados OK (FIX-O/S).");
                    } else {
                        Serial.println("[HMI] START — WARN: timeout mutex en reset de energia. Reintentando...");
                        
                        vTaskDelay(pdMS_TO_TICKS(100));
                        if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                            digitalWrite(ISOLATOR_EN_PIN, HIGH);
                            SPI.beginTransaction(adeSettings);
                            uint16_t regs_hi2[] = {0x2E7,0x323,0x35F,0x2F1,0x32D,0x369,0x2FB,0x337,0x373,
                                                   0x305,0x341,0x37D,0x30F,0x34B,0x387,0x319,0x355,0x391};
                            uint16_t regs_lo2[] = {0x2E6,0x322,0x35E,0x2F0,0x32C,0x368,0x2FA,0x336,0x372,
                                                   0x304,0x340,0x37C,0x30E,0x34A,0x386,0x318,0x354,0x390};
                            for (int i = 0; i < 18; i++) {
                                ade9000.SPI_Write_32(regs_hi2[i], 0x00000000UL);
                                ade9000.SPI_Write_32(regs_lo2[i], 0x00000000UL);
                            }
                            SPI.endTransaction();
                            xSemaphoreGive(xIsolatorMutex);
                            Serial.println("[HMI] START — Acumuladores reseteados en reintento OK.");
                        } else {
                            Serial.println("[HMI] START — ERROR: no se pudo resetear acumuladores. Energia puede iniciar en valor residual.");
                        }
                    }
                    
                    g_measuring = true;
                    Serial.println("[HMI] START — adquisicion iniciada.");
                } else {
                    hmi_beep_stop();
                    g_measuring = false;
                    digitalWrite(RELAY_PIN, RELAY_ON);
                    bitWrite(u3_state, LED_STATUS,      LED_OFF);
                    bitWrite(u2_state, LED_COMM_STATUS, LED_OFF);
                    hmi_write_u2(); hmi_write_u3();
                    Serial.println("[HMI] STOP — Adquisicion detenida. Relay ON.");
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (!g_measuring) {
                if (bitRead(pins, BTN_SETUP)) {
                    hmi_beep_short();
                    bitToggle(u2_state, LED_LOCAL_SETUP);
                    hmi_write_u2();
                }
                if (bitRead(pins, BTN_MODE)) {
                    hmi_beep_short();
                    if (bitRead(u3_state, LED_ERROR) == LED_ON) {
                        g_led_error_sd = false;
                        bitWrite(u3_state, LED_ERROR, LED_OFF);
                        hmi_write_u3();
                    }
                    bitWrite(u2_state, LED_WIFI + g_comm_mode, LED_OFF);
                    g_comm_mode = (g_comm_mode + 1) % 3;
                    bitWrite(u2_state, LED_WIFI + g_comm_mode, LED_ON);
                    hmi_write_u2();
                    Serial.printf("[HMI] MODE — Modo comm: %s\n",
                        g_comm_mode == 0 ? "WiFi" : (g_comm_mode == 1 ? "4G LTE" : "SD CARD"));
                }
                if (bitRead(pins, BTN_SYSTEM)) {
                    hmi_beep_short();
                    bitWrite(u2_state, LED_1P2W + g_system_mode, LED_OFF);
                    g_system_mode = (g_system_mode + 1) % 3;
                    bitWrite(u2_state, LED_1P2W + g_system_mode, LED_ON);
                    hmi_write_u2();
                    hmi_apply_system_mode(g_system_mode);
                    Serial.printf("[HMI] SYSTEM — Modo sistema: %s\n",
                        g_system_mode == SYS_MODE_1P2W ? "1P2W" :
                        (g_system_mode == SYS_MODE_3P4W ? "3P4W" : "3P3W"));
                }
            }
        }

        
        if (g_comm_success) { g_comm_success = false; comm_blinks_left = 6; t_comm_blink = now; }
        if (comm_blinks_left > 0 && (now - t_comm_blink) >= 40) {
            t_comm_blink = now;
            bitToggle(u2_state, LED_COMM_STATUS);
            hmi_write_u2();
            comm_blinks_left--;
            if (comm_blinks_left == 0) { bitWrite(u2_state, LED_COMM_STATUS, LED_OFF); hmi_write_u2(); }
        }

        
        {
            uint8_t disp_mode = g_failover_active ? 1 : g_comm_mode;
            if (disp_mode != led_mode_last) {
                if (led_mode_last < 3) bitWrite(u2_state, LED_WIFI + led_mode_last, LED_OFF);
                bitWrite(u2_state, LED_WIFI + disp_mode, LED_ON);
                if (disp_mode != 2) bitWrite(u2_state, LED_SD_CARD, LED_OFF);
                hmi_write_u2();
                led_mode_last = disp_mode;
            }
        }

        
        {
            if (beep_state == 0 && g_beep_request != BEEP_NONE) {
                beep_type = g_beep_request; g_beep_request = BEEP_NONE;
                beep_count = (beep_type == BEEP_ERROR) ? 3 : 1;
                beep_state = 1; t_beep = now;
                digitalWrite(BUZZER_PIN, HIGH);
            }
            if (beep_state == 1) {
                uint16_t dur = (beep_type == BEEP_ERROR) ? 80 : 600;
                if (now - t_beep >= dur) {
                    digitalWrite(BUZZER_PIN, LOW);
                    beep_count--; t_beep = now;
                    beep_state = (beep_count > 0) ? 2 : 0;
                }
            } else if (beep_state == 2) {
                if (now - t_beep >= 80) { t_beep = now; digitalWrite(BUZZER_PIN, HIGH); beep_state = 1; }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


static uint8_t bms_voltage_to_percent(float v) {
    const int N = sizeof(LIPO_VOLT_TABLE) / sizeof(LIPO_VOLT_TABLE[0]);
    if (v <= LIPO_VOLT_TABLE[0])   return 0;
    if (v >= LIPO_VOLT_TABLE[N-1]) return 100;
    for (int i = 0; i < N-1; i++) {
        if (v >= LIPO_VOLT_TABLE[i] && v < LIPO_VOLT_TABLE[i+1]) {
            float ratio = (v - LIPO_VOLT_TABLE[i]) / (LIPO_VOLT_TABLE[i+1] - LIPO_VOLT_TABLE[i]);
            return (uint8_t)(LIPO_PCT_TABLE[i] + ratio * (LIPO_PCT_TABLE[i+1] - LIPO_PCT_TABLE[i]));
        }
    }
    return 100;
}

static void bms_batt_led_breath(unsigned long now, unsigned long* t_breath, bool* breath_on) {
    unsigned long period = *breath_on ? 300 : 700;
    if (now - *t_breath >= period) {
        *t_breath = now; *breath_on = !(*breath_on);
        bitWrite(u3_state, LED_PCF_STATUS, *breath_on ? LED_ON : LED_OFF);
        pcfU3.write8(u3_state | 0x0F);
    }
}

// Core 1 task: BQ25896 battery management — voltage monitoring, low-battery cutoff.
void taskBMS(void* pvParameters) {
    Serial.println("[CORE1] taskBMS iniciado.");
    unsigned long t_last_read = 0, t_last_beep = 0, t_breath = 0, t_err_blink = 0;
    bool breath_on = false, err_led_on = false;

    while (true) {
        unsigned long now = millis();
        if (now - t_last_read < BATT_UPDATE_MS) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        t_last_read = now;

        if (!bq25896.isConnected()) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

        
        auto vbus_reg  = bq25896.get_VBUS_STAT_reg();
        auto fault_reg = bq25896.getFAULT_reg();

        uint8_t chrg_stat  = vbus_reg.chrg_stat;
        uint8_t vbus_stat  = vbus_reg.vbus_stat;
        uint8_t chrg_fault = fault_reg.chrg_fault;

        float   vbat = (float)bq25896.getBATV() / 1000.0f;

        
        static float   vbat_buf[10] = {0.0f};
        static uint8_t vbat_idx     = 0;
        static bool    vbat_init    = false;
        if (!vbat_init) {
            for (int i = 0; i < 10; i++) vbat_buf[i] = vbat;
            vbat_init = true;
        }
        vbat_buf[vbat_idx % 10] = vbat;
        vbat_idx++;
        float vbat_avg = 0.0f;
        for (int i = 0; i < 10; i++) vbat_avg += vbat_buf[i];
        vbat_avg /= 10.0f;

        uint8_t pct  = bms_voltage_to_percent(vbat_avg);

        g_batt_voltage    = vbat_avg;
        g_batt_percent    = pct;
        g_batt_charging   = (chrg_stat == 1 || chrg_stat == 2);
        g_batt_full       = (chrg_stat == 3);
        g_batt_on_battery = (vbus_stat == 0);
        g_batt_error      = (chrg_fault != 0) && (vbus_stat != 0);

        if (g_batt_error) {
            if (now - t_err_blink >= 250) {
                t_err_blink = now; err_led_on = !err_led_on;
                bitWrite(u3_state, LED_ERROR,      err_led_on ? LED_ON : LED_OFF);
                bitWrite(u3_state, LED_PCF_STATUS, LED_OFF);
                pcfU3.write8(u3_state | 0x0F);
                digitalWrite(BUZZER_PIN, err_led_on ? HIGH : LOW);
            }
        } else {
            if (!g_led_error_sd && bitRead(u3_state, LED_ERROR) == LED_ON) {
                bitWrite(u3_state, LED_ERROR, LED_OFF);
                digitalWrite(BUZZER_PIN, LOW);
                pcfU3.write8(u3_state | 0x0F);
            }
            if (g_batt_charging) {
                bms_batt_led_breath(now, &t_breath, &breath_on);
            } else if (g_batt_full) {
                bitWrite(u3_state, LED_PCF_STATUS, LED_OFF);
                pcfU3.write8(u3_state | 0x0F);
            } else if (g_batt_on_battery) {
                bitWrite(u3_state, LED_PCF_STATUS, LED_ON);
                pcfU3.write8(u3_state | 0x0F);
                
                if (vbat_avg <= BATT_LOW_VOLT) {
                    
                    if (now - t_breath >= 200) {
                        t_breath = now; breath_on = !breath_on;
                        bitWrite(u3_state, LED_PCF_STATUS, breath_on ? LED_ON : LED_OFF);
                        pcfU3.write8(u3_state | 0x0F);
                    }
                    
                    if (now - t_last_beep >= BATT_LOW_BEEP_INTERVAL_MS) {
                        t_last_beep = now;
                        for (int i = 0; i < 3; i++) {
                            digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
                            digitalWrite(BUZZER_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
                        }
                        Serial.printf("[BMS] BATERIA BAJA! %.2fV (avg) = %d%%\n", vbat_avg, pct);
                    }
                }

                
                if (vbat_avg > 0.5f && vbat_avg <= 3.0f) {
                    Serial.printf("[BMS] CORTE BATERIA: promedio=%.3fV <= 3.0V — activando BATFET_DIS\n", vbat_avg);
                    bq25896.setBATFET_DIS(true);
                    
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            } else {
                bitWrite(u3_state, LED_PCF_STATUS, LED_OFF);
                pcfU3.write8(u3_state | 0x0F);
            }
        }

        static unsigned long t_log = 0;
        if (now - t_log > 30000) {
            t_log = now;
            Serial.printf("[BMS] Vbat=%.2fV (%d%%) chrg=%d vbus=%d fault=0x%02X %s\n",
                          vbat, pct, chrg_stat, vbus_stat, chrg_fault,
                          g_batt_on_battery ? "BATERIA" : "RED");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


WiFiClientSecure    wifiClientSecure;
WiFiMulti           wifiMulti;
PubSubClient        mqttWifi(wifiClientSecure);
HardwareSerial      SerialGSM(2);
TinyGsm             modem(SerialGSM);
TinyGsmClient       gsmClient(modem);
Arduino_MQTT_Client mqttClient4G(gsmClient);
ThingsBoardSized<8, 75> tb(mqttClient4G, 2048, 2048);
PubSubClient* mqttActive = &mqttWifi;

static inline float round6(float v) { return roundf(v * 1000000.0f) / 1000000.0f; }


static void publishStartAttributes(bool use4G) {
    float v_nom = (g_system_mode == SYS_MODE_3P3W) ? 220.0f : 127.0f;
    g_v_nom = v_nom;
    g_v_min = roundf(v_nom * 0.90f * 10.0f) / 10.0f;
    g_v_max = roundf(v_nom * 1.10f * 10.0f) / 10.0f;
    uint32_t ts = getUnixTS();

    if (!use4G) {
        StaticJsonDocument<1800> doc;
        doc["sys_mode"]  = (int)g_system_mode;
        doc["comm_mode"] = (int)(g_failover_active ? 1 : g_comm_mode);
        doc["phase_seq"] = (int)g_phase_seq;
        doc["v_nom"]     = g_v_nom;
        doc["v_min"]     = g_v_min;
        doc["v_max"]     = g_v_max;
        doc["ts"]        = (uint64_t)ts * 1000ULL;  
        if (g_system_mode == SYS_MODE_1P2W) {
            doc["Vrms_B"]=0.0f; doc["Vrms_C"]=0.0f; doc["Irms_B"]=0.0f; doc["Irms_C"]=0.0f;
            doc["Pa_B"]=0.0f;   doc["Pa_C"]=0.0f;   doc["Pr_B"]=0.0f;   doc["Pr_C"]=0.0f;
            doc["Ps_B"]=0.0f;   doc["Ps_C"]=0.0f;   doc["PF_B"]=0.0f;   doc["PF_C"]=0.0f;
            doc["THDv_B"]=0.0f; doc["THDv_C"]=0.0f; doc["THDi_B"]=0.0f; doc["THDi_C"]=0.0f;
            doc["freq_B"]=0.0f; doc["freq_C"]=0.0f; doc["Vh_B"]=0.0f;   doc["Vh_C"]=0.0f;
            doc["V_unb"]=0.0f;  doc["I_unb"]=0.0f;  doc["Eact_B"]=0.0f; doc["Eact_C"]=0.0f;
            doc["Vf_B"]=0.0f;   doc["Vf_C"]=0.0f;   doc["If_B"]=0.0f;   doc["If_C"]=0.0f;
            doc["ang_AB"]=0.0f; doc["ang_BC"]=0.0f; doc["ang_AC"]=0.0f;
            doc["ang_B"]=0.0f;  doc["ang_C"]=0.0f;
            doc["ang_IA_IB"]=0.0f; doc["ang_IB_IC"]=0.0f; doc["ang_IA_IC"]=0.0f;
        }
        if (g_system_mode == SYS_MODE_3P3W) doc["Irms_N"] = 0.0f;
        char buf[700];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        if (len > 0 && mqttActive && mqttActive->connected()) {
            mqttActive->publish(TB_TOPIC, buf, false);
            Serial.printf("[COMM] Atributos START enviados via WiFi (%u bytes)\n", (unsigned)len);
        }
    } else {
        if (!tb.connected()) return;
        const Telemetry attrs_base[] = {
            Telemetry("sys_mode",  (int)g_system_mode),
            Telemetry("comm_mode", (int)(g_failover_active ? 1 : g_comm_mode)),
            Telemetry("phase_seq", (int)g_phase_seq),
            Telemetry("v_nom",     g_v_nom),
            Telemetry("v_min",     g_v_min),
            Telemetry("v_max",     g_v_max),
        };
        tb.sendTelemetry<6>(std::begin(attrs_base), std::end(attrs_base));
        if (g_system_mode == SYS_MODE_1P2W) {
            const Telemetry zeros[] = {
                Telemetry("Vrms_B",0.0f), Telemetry("Vrms_C",0.0f), Telemetry("Irms_B",0.0f), Telemetry("Irms_C",0.0f),
                Telemetry("Pa_B",0.0f),   Telemetry("Pa_C",0.0f),   Telemetry("Pr_B",0.0f),   Telemetry("Pr_C",0.0f),
                Telemetry("Ps_B",0.0f),   Telemetry("Ps_C",0.0f),   Telemetry("PF_B",0.0f),   Telemetry("PF_C",0.0f),
                Telemetry("THDv_B",0.0f), Telemetry("THDv_C",0.0f), Telemetry("THDi_B",0.0f), Telemetry("THDi_C",0.0f),
                Telemetry("freq_B",0.0f), Telemetry("freq_C",0.0f), Telemetry("Vh_B",0.0f),   Telemetry("Vh_C",0.0f),
                Telemetry("V_unb",0.0f),  Telemetry("I_unb",0.0f),  Telemetry("Eact_B",0.0f), Telemetry("Eact_C",0.0f),
                Telemetry("Vf_B",0.0f),   Telemetry("Vf_C",0.0f),   Telemetry("If_B",0.0f),   Telemetry("If_C",0.0f),
                Telemetry("ang_AB",0.0f), Telemetry("ang_BC",0.0f), Telemetry("ang_AC",0.0f),
                Telemetry("ang_B",0.0f),  Telemetry("ang_C",0.0f),
                Telemetry("ang_IA_IB",0.0f), Telemetry("ang_IB_IC",0.0f), Telemetry("ang_IA_IC",0.0f),
            };
            tb.sendTelemetry<36>(std::begin(zeros), std::end(zeros));
        }
        if (g_system_mode == SYS_MODE_3P3W) {
            const Telemetry neutral_zero[] = { Telemetry("Irms_N", 0.0f) };
            tb.sendTelemetry<1>(std::begin(neutral_zero), std::end(neutral_zero));
        }
        Serial.println("[COMM] Atributos START enviados via 4G");
    }

    if (!g_gps_fix) {
        g_gps_lat = 19.407365f; g_gps_lon = -98.119003f;
        Serial.println("[COMM] GPS: sin fix — enviando coordenadas universidad como posicion inicial");
        if (!use4G) {
            StaticJsonDocument<128> doc;
            doc["gps_lat"] = 19.407365f; doc["gps_lon"] = -98.119003f;
            char buf[128]; size_t len = serializeJson(doc, buf, sizeof(buf));
            if (len > 0 && mqttActive && mqttActive->connected()) mqttActive->publish(TB_TOPIC, buf, false);
        } else {
            if (tb.connected()) {
                const Telemetry gps_default[] = { Telemetry("gps_lat", 19.407365f), Telemetry("gps_lon", -98.119003f) };
                tb.sendTelemetry<2>(std::begin(gps_default), std::end(gps_default));
            }
        }
    }
    g_start_attrs_sent = true;
}


// Serializes and publishes a 3-second record to ThingsBoard via WiFi MQTT.
static bool publishTelemetry(const IEC_150_180_Record& r, bool include_ts = true) {
    StaticJsonDocument<2048> doc;
    IEC_150_180_Record m = r;
    bool is_1p2w = (g_system_mode == SYS_MODE_1P2W);
    bool is_3p3w = (g_system_mode == SYS_MODE_3P3W);

    doc["Vrms_A"] = roundf(m.vrms_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vrms_B"] = roundf(m.vrms_b * 1000.0f) / 1000.0f; doc["Vrms_C"] = roundf(m.vrms_c * 1000.0f) / 1000.0f; }
    if (!is_1p2w) doc["V_unb"] = roundf(m.u2_voltage * 100.0f) / 100.0f;
    doc["Vh_A"] = roundf(m.vrms_half_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vh_B"] = roundf(m.vrms_half_b * 1000.0f) / 1000.0f; doc["Vh_C"] = roundf(m.vrms_half_c * 1000.0f) / 1000.0f; }
    doc["freq_A"] = roundf(m.freq_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["freq_B"] = roundf(m.freq_b * 100.0f) / 100.0f; doc["freq_C"] = roundf(m.freq_c * 100.0f) / 100.0f; }
    doc["Irms_A"] = roundf(m.irms_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Irms_B"] = roundf(m.irms_b * 1000.0f) / 1000.0f; doc["Irms_C"] = roundf(m.irms_c * 1000.0f) / 1000.0f; }
    if (!is_3p3w) doc["Irms_N"] = roundf(m.irms_n * 1000.0f) / 1000.0f;
    if (!is_1p2w) doc["I_unb"] = roundf(m.u2_current * 100.0f) / 100.0f;
    doc["Vf_A"] = roundf(m.vrms_fund_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vf_B"] = roundf(m.vrms_fund_b * 1000.0f) / 1000.0f; doc["Vf_C"] = roundf(m.vrms_fund_c * 1000.0f) / 1000.0f; }
    doc["If_A"] = roundf(m.irms_fund_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["If_B"] = roundf(m.irms_fund_b * 1000.0f) / 1000.0f; doc["If_C"] = roundf(m.irms_fund_c * 1000.0f) / 1000.0f; }
    doc["Pa_A"] = roundf(m.pa_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Pa_B"] = roundf(m.pa_b * 100.0f) / 100.0f; doc["Pa_C"] = roundf(m.pa_c * 100.0f) / 100.0f; }
    doc["Pr_A"] = roundf(m.pr_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Pr_B"] = roundf(m.pr_b * 100.0f) / 100.0f; doc["Pr_C"] = roundf(m.pr_c * 100.0f) / 100.0f; }
    doc["Ps_A"] = roundf(m.papp_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Ps_B"] = roundf(m.papp_b * 100.0f) / 100.0f; doc["Ps_C"] = roundf(m.papp_c * 100.0f) / 100.0f; }
    doc["PF_A"] = roundf(m.pf_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["PF_B"] = roundf(m.pf_b * 1000.0f) / 1000.0f; doc["PF_C"] = roundf(m.pf_c * 1000.0f) / 1000.0f; }
    doc["THDv_A"] = roundf(m.thd_v_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["THDv_B"] = roundf(m.thd_v_b * 100.0f) / 100.0f; doc["THDv_C"] = roundf(m.thd_v_c * 100.0f) / 100.0f; }
    doc["THDi_A"] = roundf(m.thd_i_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["THDi_B"] = roundf(m.thd_i_b * 100.0f) / 100.0f; doc["THDi_C"] = roundf(m.thd_i_c * 100.0f) / 100.0f; }
    if (!is_1p2w) {
        doc["ang_AB"]    = roundf(m.angle_va_vb * 10.0f) / 10.0f;
        doc["ang_BC"]    = roundf(m.angle_vb_vc * 10.0f) / 10.0f;
        doc["ang_AC"]    = roundf(m.angle_va_vc * 10.0f) / 10.0f;
        doc["ang_B"]     = roundf(m.angle_vb_ib * 10.0f) / 10.0f;
        doc["ang_C"]     = roundf(m.angle_vc_ic * 10.0f) / 10.0f;
        doc["ang_IA_IB"] = roundf(m.angle_ia_ib * 10.0f) / 10.0f;
        doc["ang_IB_IC"] = roundf(m.angle_ib_ic * 10.0f) / 10.0f;
        doc["ang_IA_IC"] = roundf(m.angle_ia_ic * 10.0f) / 10.0f;
    }
    doc["ang_A"] = roundf(m.angle_va_ia * 10.0f) / 10.0f;

    doc["evt_sag"]          = g_evt_sag          ? 1 : 0;
    doc["evt_swell"]        = g_evt_swell         ? 1 : 0;
    doc["evt_interruption"] = g_evt_interruption  ? 1 : 0;
    
    if (g_evt_sag) {
        char ph[2] = { g_evt_sag_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_sag_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_sag_ts * 1000ULL;
        doc["evt_residual_v"]  = roundf(g_evt_sag_res * 1000.0f) / 1000.0f;
    }
    if (g_evt_interruption) {
        char ph[2] = { g_evt_int_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_int_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_int_ts * 1000ULL;
        doc["evt_residual_v"]  = roundf(g_evt_int_res * 1000.0f) / 1000.0f;
    }
    if (g_evt_swell) {
        char ph[2] = { g_evt_swl_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_swl_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_swl_ts * 1000ULL;
        doc["evt_peak_v"]      = roundf(g_evt_swl_peak * 1000.0f) / 1000.0f;
    }
    g_evt_sag = g_evt_swell = g_evt_interruption = false;
    g_evt_sag_phase='-'; g_evt_sag_ts=0; g_evt_sag_dur=0; g_evt_sag_res=0.0f;
    g_evt_int_phase='-'; g_evt_int_ts=0; g_evt_int_dur=0; g_evt_int_res=0.0f;
    g_evt_swl_phase='-'; g_evt_swl_ts=0; g_evt_swl_dur=0; g_evt_swl_peak=0.0f;

    doc["oc_flag"]         = (int)g_oc_flag;
    doc["temp_ade"]        = roundf(m.temp_chip * 10.0f) / 10.0f;
    doc["batt_v"]          = roundf(g_batt_voltage * 100.0f) / 100.0f;
    doc["batt_pct"]        = g_batt_percent;
    doc["rssi_wifi"]       = (int)WiFi.RSSI();
    { int csq4g = modem.getSignalQuality(); doc["rssi_4g"] = (csq4g <= 31) ? (-113.0f + csq4g * 2.0f) : -999.0f; }
    doc["comm_mode"]       = (int)(g_failover_active ? 1 : g_comm_mode);
    doc["failover_active"] = g_failover_active ? 1 : 0;
    
    
    if (include_ts && m.timestamp > 0) {
        doc["ts"] = m.timestamp;
    }

    char payload[2048];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0 || len >= sizeof(payload)) { Serial.println("[COMM] ERROR: serializeJson fallo"); return false; }
    bool ok = mqttActive->publish(TB_TOPIC, payload, false);
    if (ok) { g_tx_messages++; g_tx_msg_wifi++; g_tx_bytes_wifi += (uint64_t)len; Serial.printf("[WIFI] TX OK — %u bytes | RSSI: %d dBm\n", (unsigned)len, WiFi.RSSI()); }
    else    { Serial.printf("[WIFI] TX FAIL | RSSI: %d dBm\n", WiFi.RSSI()); }
    return ok;
}


static void publishRealAlarm(const char* alarm_type, StaticJsonDocument<256>& alarm_doc,
                              bool use4G) {
    
    uint64_t msg_ts = 0;
    if (alarm_doc.containsKey("ts")) {
        msg_ts = alarm_doc["ts"].as<uint64_t>();
        alarm_doc.remove("ts");
    } else {
        msg_ts = (uint64_t)getUnixTS() * 1000ULL;
    }

    
    StaticJsonDocument<384> envelope;
    envelope["ts"] = msg_ts;
    JsonObject values = envelope.createNestedObject("values");
    for (JsonPair kv : alarm_doc.as<JsonObject>()) {
        values[kv.key()] = kv.value();
    }

    char alarm_buf[384];
    size_t alarm_len = serializeJson(envelope, alarm_buf, sizeof(alarm_buf));
    if (alarm_len == 0) return;

    if (use4G) {
        if (tb.connected()) tb.sendTelemetryString(alarm_buf);
    } else {
        if (mqttActive && mqttActive->connected()) mqttActive->publish(TB_TOPIC, alarm_buf, false);
    }
    Serial.printf("[ALARM] %s enviada\n", alarm_type);
}


// Serializes and publishes a 3-second record to ThingsBoard via 4G.
static bool publishTelemetry4G(const IEC_150_180_Record& r, bool include_ts = true) {
    bool is_1p2w = (g_system_mode == SYS_MODE_1P2W);
    bool is_3p3w = (g_system_mode == SYS_MODE_3P3W);
    int csq = modem.getSignalQuality();
    float rssi_4g = (csq <= 31) ? (-113.0f + csq * 2.0f) : -999.0f;

    StaticJsonDocument<2048> doc;
    doc["Vrms_A"] = roundf(r.vrms_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vrms_B"] = roundf(r.vrms_b * 1000.0f) / 1000.0f; doc["Vrms_C"] = roundf(r.vrms_c * 1000.0f) / 1000.0f; doc["V_unb"] = roundf(r.u2_voltage * 100.0f) / 100.0f; }
    doc["Vh_A"] = roundf(r.vrms_half_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vh_B"] = roundf(r.vrms_half_b * 1000.0f) / 1000.0f; doc["Vh_C"] = roundf(r.vrms_half_c * 1000.0f) / 1000.0f; }
    doc["freq_A"] = roundf(r.freq_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["freq_B"] = roundf(r.freq_b * 100.0f) / 100.0f; doc["freq_C"] = roundf(r.freq_c * 100.0f) / 100.0f; }
    doc["Irms_A"] = roundf(r.irms_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Irms_B"] = roundf(r.irms_b * 1000.0f) / 1000.0f; doc["Irms_C"] = roundf(r.irms_c * 1000.0f) / 1000.0f; doc["I_unb"] = roundf(r.u2_current * 100.0f) / 100.0f; }
    if (!is_3p3w) doc["Irms_N"] = roundf(r.irms_n * 1000.0f) / 1000.0f;
    doc["Vf_A"] = roundf(r.vrms_fund_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["Vf_B"] = roundf(r.vrms_fund_b * 1000.0f) / 1000.0f; doc["Vf_C"] = roundf(r.vrms_fund_c * 1000.0f) / 1000.0f; }
    doc["If_A"] = roundf(r.irms_fund_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["If_B"] = roundf(r.irms_fund_b * 1000.0f) / 1000.0f; doc["If_C"] = roundf(r.irms_fund_c * 1000.0f) / 1000.0f; }
    doc["Pa_A"] = roundf(r.pa_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Pa_B"] = roundf(r.pa_b * 100.0f) / 100.0f; doc["Pa_C"] = roundf(r.pa_c * 100.0f) / 100.0f; }
    doc["Pr_A"] = roundf(r.pr_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Pr_B"] = roundf(r.pr_b * 100.0f) / 100.0f; doc["Pr_C"] = roundf(r.pr_c * 100.0f) / 100.0f; }
    doc["Ps_A"] = roundf(r.papp_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["Ps_B"] = roundf(r.papp_b * 100.0f) / 100.0f; doc["Ps_C"] = roundf(r.papp_c * 100.0f) / 100.0f; }
    doc["PF_A"] = roundf(r.pf_a * 1000.0f) / 1000.0f;
    if (!is_1p2w) { doc["PF_B"] = roundf(r.pf_b * 1000.0f) / 1000.0f; doc["PF_C"] = roundf(r.pf_c * 1000.0f) / 1000.0f; }
    doc["THDv_A"] = roundf(r.thd_v_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["THDv_B"] = roundf(r.thd_v_b * 100.0f) / 100.0f; doc["THDv_C"] = roundf(r.thd_v_c * 100.0f) / 100.0f; }
    doc["THDi_A"] = roundf(r.thd_i_a * 100.0f) / 100.0f;
    if (!is_1p2w) { doc["THDi_B"] = roundf(r.thd_i_b * 100.0f) / 100.0f; doc["THDi_C"] = roundf(r.thd_i_c * 100.0f) / 100.0f; }
    doc["ang_A"] = roundf(r.angle_va_ia * 10.0f) / 10.0f;
    if (!is_1p2w) {
        doc["ang_AB"] = roundf(r.angle_va_vb * 10.0f) / 10.0f; doc["ang_BC"] = roundf(r.angle_vb_vc * 10.0f) / 10.0f;
        doc["ang_AC"] = roundf(r.angle_va_vc * 10.0f) / 10.0f; doc["ang_B"]  = roundf(r.angle_vb_ib * 10.0f) / 10.0f;
        doc["ang_C"]  = roundf(r.angle_vc_ic * 10.0f) / 10.0f;
        doc["ang_IA_IB"] = roundf(r.angle_ia_ib * 10.0f) / 10.0f; doc["ang_IB_IC"] = roundf(r.angle_ib_ic * 10.0f) / 10.0f;
        doc["ang_IA_IC"] = roundf(r.angle_ia_ic * 10.0f) / 10.0f;
    }

    
    doc["evt_sag"]          = g_evt_sag          ? 1 : 0;
    doc["evt_swell"]        = g_evt_swell         ? 1 : 0;
    doc["evt_interruption"] = g_evt_interruption  ? 1 : 0;
    
    if (g_evt_sag) {
        char ph[2] = { g_evt_sag_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_sag_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_sag_ts * 1000ULL;
        doc["evt_residual_v"]  = roundf(g_evt_sag_res * 1000.0f) / 1000.0f;
    }
    if (g_evt_interruption) {
        char ph[2] = { g_evt_int_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_int_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_int_ts * 1000ULL;
        doc["evt_residual_v"]  = roundf(g_evt_int_res * 1000.0f) / 1000.0f;
    }
    if (g_evt_swell) {
        char ph[2] = { g_evt_swl_phase, '\0' };
        doc["evt_phase"]       = ph;
        doc["evt_duration_ms"] = g_evt_swl_dur;
        doc["evt_ts_start"]    = (uint64_t)g_evt_swl_ts * 1000ULL;
        doc["evt_peak_v"]      = roundf(g_evt_swl_peak * 1000.0f) / 1000.0f;
    }
    g_evt_sag = g_evt_swell = g_evt_interruption = false;
    g_evt_sag_phase='-'; g_evt_sag_ts=0; g_evt_sag_dur=0; g_evt_sag_res=0.0f;
    g_evt_int_phase='-'; g_evt_int_ts=0; g_evt_int_dur=0; g_evt_int_res=0.0f;
    g_evt_swl_phase='-'; g_evt_swl_ts=0; g_evt_swl_dur=0; g_evt_swl_peak=0.0f;

    doc["oc_flag"]         = (int)g_oc_flag;
    doc["temp_ade"]        = roundf(r.temp_chip * 10.0f) / 10.0f;
    doc["batt_v"]          = roundf(g_batt_voltage * 100.0f) / 100.0f;
    doc["batt_pct"]        = (int)g_batt_percent;
    doc["rssi_wifi"]       = (int)WiFi.RSSI();
    doc["rssi_4g"]         = roundf(rssi_4g * 10.0f) / 10.0f;
    doc["comm_mode"]       = (int)(g_failover_active ? 1 : g_comm_mode);
    doc["failover_active"] = (int)(g_failover_active ? 1 : 0);
    
    if (include_ts && r.timestamp > 0) {
        doc["ts"] = r.timestamp;
    }

    char json[2048];
    size_t len = serializeJson(doc, json, sizeof(json));
    if (len == 0) return false;
    bool ok = tb.sendTelemetryString(json);
    if (ok) { g_tx_messages++; g_tx_msg_4g++; g_tx_bytes_4g += (uint64_t)len; Serial.printf("[4G] TX OK — %d bytes | CSQ: %d\n", (int)len, csq); }
    else    { Serial.printf("[4G] TX FAIL | CSQ: %d\n", csq); }
    return ok;
}


static void publishOCAlarm(bool use4G) {
    if (!g_oc_flag) return;
    Serial.printf("[OC-ALARM] Sobrecorriente: fases=0x%02X A=%.1fA B=%.1fA C=%.1fA\n",
                  g_oc_phase, g_oc_peak_a, g_oc_peak_b, g_oc_peak_c);
    if (use4G) {
        if (!tb.connected()) return;
        const Telemetry oc[] = { Telemetry("oc_phase",(int)g_oc_phase), Telemetry("oc_peak_a",g_oc_peak_a), Telemetry("oc_peak_b",g_oc_peak_b), Telemetry("oc_peak_c",g_oc_peak_c) };
        tb.sendTelemetry<4>(std::begin(oc), std::end(oc));
    } else {
        if (!mqttActive || !mqttActive->connected()) return;
        StaticJsonDocument<256> doc;
        doc["oc_phase"]=g_oc_phase; doc["oc_peak_a"]=g_oc_peak_a; doc["oc_peak_b"]=g_oc_peak_b; doc["oc_peak_c"]=g_oc_peak_c;
        char buf[256]; size_t len = serializeJson(doc, buf, sizeof(buf));
        mqttActive->publish(TB_TOPIC, buf, false);
    }
}


static bool publishGPS(bool use4G) {
    if (!g_gps_fix) { Serial.printf("[%s][GPS] Sin fix — omitiendo publicacion GPS\n", getTimeStr()); return false; }
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        digitalWrite(ISOLATOR_EN_PIN, LOW); delayMicroseconds(500);  
        File fg = SD.open("/GPS_LOG.csv", FILE_APPEND);
        if (fg) {
            if (fg.size() == 0) fg.println("ts,lat,lon,fix,hdop");
            fg.printf("%u,%.6f,%.6f,%d,%.1f\n", getUnixTS(), g_gps_lat, g_gps_lon, (int)g_gps_fix, g_gps_hdop);
            fg.flush(); fg.close();
        }
        digitalWrite(ISOLATOR_EN_PIN, HIGH);
        xSemaphoreGive(xIsolatorMutex);
    }
    if (use4G) {
        if (!tb.connected()) return false;
        const Telemetry gps_data[] = { Telemetry("gps_lat",g_gps_lat), Telemetry("gps_lon",g_gps_lon), Telemetry("gps_fix",(int)g_gps_fix) };
        bool ok = tb.sendTelemetry<3>(std::begin(gps_data), std::end(gps_data));
        if (ok) Serial.printf("[%s][GPS] Publicado via 4G: Lat=%.6f Lon=%.6f\n", getTimeStr(), g_gps_lat, g_gps_lon);
        return ok;
    } else {
        if (!mqttActive || !mqttActive->connected()) return false;
        StaticJsonDocument<128> doc;
        doc["gps_lat"]=round6(g_gps_lat); doc["gps_lon"]=round6(g_gps_lon); doc["gps_fix"]=(int)g_gps_fix; doc["ts"]=(uint64_t)getUnixTS()*1000ULL;
        char buf[128]; size_t len = serializeJson(doc, buf, sizeof(buf));
        bool ok = mqttActive->publish(TB_TOPIC, buf, false);
        if (ok) Serial.printf("[%s][GPS] Publicado via WiFi: Lat=%.6f Lon=%.6f\n", getTimeStr(), g_gps_lat, g_gps_lon);
        return ok;
    }
}


static bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    Serial.println("[COMM] Conectando WiFi (multi-red)...");
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_PASS_2);
    uint32_t t0 = millis();
    while (wifiMulti.run() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) vTaskDelay(pdMS_TO_TICKS(100));
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[COMM] WiFi OK — SSID: %s | IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        return true;
    }
    Serial.println("[COMM] WiFi TIMEOUT — red no disponible");
    return false;
}

// Connects PubSubClient to ThingsBoard over TLS (port 8883).
static bool connectMQTT() {
    mqttActive = &mqttWifi;
    if (mqttActive->connected()) return true;
    mqttActive->setServer(TB_SERVER, TB_PORT);
    mqttActive->setBufferSize(2048, 2048);
    Serial.printf("[WIFI] Conectando MQTTS a %s:%d...\n", TB_SERVER, TB_PORT);
    if (mqttActive->connect("ITA9000", TB_TOKEN, NULL)) { Serial.println("[WIFI] MQTT OK"); return true; }
    Serial.printf("[WIFI] MQTT FAIL — estado: %d\n", mqttActive->state());
    return false;
}


// Brings up modem, waits for network registration, connects GPRS.
static bool connect4G() {
    if (modem.isNetworkConnected() && modem.isGprsConnected()) return true;
    Serial.println("[4G] Iniciando modem...");
    digitalWrite(GSM_SLEEP_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(GSM_SLEEP_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(1000));
    bool modem_alive = false;
    for (int i = 0; i < 5; i++) {
        SerialGSM.println("AT"); vTaskDelay(pdMS_TO_TICKS(500));
        String resp = ""; uint32_t t0 = millis();
        while (millis() - t0 < 500) { while (SerialGSM.available()) resp += (char)SerialGSM.read(); }
        if (resp.indexOf("OK") >= 0) { modem_alive = true; Serial.println("[4G] Modem responde AT"); break; }
        Serial.printf("[4G] Intento %d/5 — sin respuesta\n", i+1);
    }
    if (modem_alive) {
        SerialGSM.println("AT+CSCLK=0"); vTaskDelay(pdMS_TO_TICKS(500));
        while (SerialGSM.available()) SerialGSM.read();
        if (!modem.init()) { Serial.println("[4G] init() fallo"); return false; }
    } else {
        Serial.println("[4G] Modem no responde — intentando restart()...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!modem.restart()) { Serial.println("[4G] restart() fallo"); return false; }
        SerialGSM.println("AT+CSCLK=0"); vTaskDelay(pdMS_TO_TICKS(500));
        while (SerialGSM.available()) SerialGSM.read();
    }
    Serial.println("[4G] Modem OK");
    if (!modem.waitForNetwork(30000)) { Serial.println("[4G] Sin red"); return false; }
    Serial.printf("[4G] Conectando GPRS APN: %s...\n", GSM_APN);
    if (!modem.gprsConnect(GSM_APN)) { Serial.println("[4G] GPRS fallo"); return false; }
    Serial.printf("[4G] OK — IP: %s\n", modem.getLocalIP().c_str());
    return true;
}


// NTP sync over WiFi using ESP32 configTime / getLocalTime.
static void syncNTP() {
    Serial.println("[COMM] Sincronizando NTP via WiFi...");
    configTime(NTP_GMT_OFFSET_SEC, NTP_DST_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
    struct tm timeinfo;
    uint32_t t0 = millis();
    while (!getLocalTime(&timeinfo) && (millis() - t0) < 10000) vTaskDelay(pdMS_TO_TICKS(500));
    if (getLocalTime(&timeinfo)) {
        DateTime ntpTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        rtc.adjust(ntpTime);
        g_last_ntp_sync     = rtc.now().unixtime();
        
        g_ts_epoch_ms      = (uint64_t)g_last_ntp_sync * 1000ULL;
        g_ts_millis_anchor = millis();
        Serial.printf("[COMM] NTP WiFi OK — %02d:%02d:%02d UTC\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else { Serial.println("[COMM] NTP WiFi TIMEOUT"); }
}

// NTP sync over 4G data using AT+CNTP command, then reads AT+CCLK?.
static bool syncNTP_4G() {
    Serial.println("[COMM] NTP via 4G...");
    SerialGSM.println("AT+CNTP=\"pool.ntp.org\",0"); vTaskDelay(pdMS_TO_TICKS(500));
    while (SerialGSM.available()) SerialGSM.read();
    SerialGSM.println("AT+CNTP"); vTaskDelay(pdMS_TO_TICKS(3000));
    String resp = ""; uint32_t t0 = millis();
    while (millis() - t0 < 3000) { if (SerialGSM.available()) resp += (char)SerialGSM.read(); }
    if (resp.indexOf("+CNTP: 1") >= 0 || resp.indexOf("OK") >= 0) {
        while (SerialGSM.available()) SerialGSM.read();
        SerialGSM.println("AT+CCLK?"); vTaskDelay(pdMS_TO_TICKS(500));
        String cclk = ""; t0 = millis();
        while (millis() - t0 < 1000) { if (SerialGSM.available()) cclk += (char)SerialGSM.read(); }
        int idx = cclk.indexOf("+CCLK: \"");
        if (idx >= 0) {
            String dt = cclk.substring(idx + 8);
            int yy=dt.substring(0,2).toInt()+2000, mon=dt.substring(3,5).toInt(), dd=dt.substring(6,8).toInt();
            int hh=dt.substring(9,11).toInt(), mm=dt.substring(12,14).toInt(), ss=dt.substring(15,17).toInt();
            if (yy >= 2024 && mon >= 1 && mon <= 12 && dd >= 1) {
                DateTime t4g(yy, mon, dd, hh, mm, ss);
                rtc.adjust(t4g);
                g_last_ntp_sync    = rtc.now().unixtime();
                
                g_ts_epoch_ms      = (uint64_t)g_last_ntp_sync * 1000ULL;
                g_ts_millis_anchor = millis();
                Serial.printf("[COMM] NTP 4G OK — %04d-%02d-%02d %02d:%02d:%02d UTC\n", yy, mon, dd, hh, mm, ss);
                return true;
            }
        }
    }
    Serial.println("[COMM] NTP 4G FAIL");
    return false;
}


// NITZ time sync via A7670SA — no data connection required, only network registration.
// Wakes modem, reads AT+CCLK?, converts local time to UTC, anchors epoch.
// In SD-only mode (g_comm_mode==2) puts modem back to sleep after sync.
static bool syncNITZ() {
    Serial.println("[NITZ] Iniciando sincronizacion NITZ (red celular)...");

    
    digitalWrite(GSM_SLEEP_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(GSM_SLEEP_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(1000));
    while (SerialGSM.available()) SerialGSM.read();

    
    bool modem_alive = false;
    for (int i = 0; i < 5; i++) {
        SerialGSM.println("AT"); vTaskDelay(pdMS_TO_TICKS(500));
        String resp = ""; uint32_t t0 = millis();
        while (millis() - t0 < 500) { if (SerialGSM.available()) resp += (char)SerialGSM.read(); }
        if (resp.indexOf("OK") >= 0) { modem_alive = true; break; }
        Serial.printf("[NITZ] AT intento %d/5 sin respuesta\n", i + 1);
    }
    if (!modem_alive) {
        Serial.println("[NITZ] Modem no responde — NITZ abortado");
        return false;
    }
    SerialGSM.println("AT+CSCLK=0"); vTaskDelay(pdMS_TO_TICKS(500));
    while (SerialGSM.available()) SerialGSM.read();

    
    while (SerialGSM.available()) SerialGSM.read();
    SerialGSM.println("AT+CREG?"); vTaskDelay(pdMS_TO_TICKS(500));
    String resp = ""; uint32_t t0 = millis();
    while (millis() - t0 < 1000) { if (SerialGSM.available()) resp += (char)SerialGSM.read(); }
    if (resp.indexOf(",1") < 0 && resp.indexOf(",5") < 0) {
        Serial.println("[NITZ] Sin registro en red — NITZ no disponible");
        
        if (g_comm_mode == 2) {
            SerialGSM.println("AT+CSCLK=2"); vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(GSM_SLEEP_PIN, HIGH);
            Serial.println("[NITZ] Modem a sleep (SD-only, sin red)");
        }
        return false;
    }

    
    while (SerialGSM.available()) SerialGSM.read();
    SerialGSM.println("AT+CCLK?"); vTaskDelay(pdMS_TO_TICKS(500));
    resp = ""; t0 = millis();
    while (millis() - t0 < 1000) { if (SerialGSM.available()) resp += (char)SerialGSM.read(); }

    int idx = resp.indexOf("+CCLK: \"");
    if (idx < 0) idx = resp.indexOf("+CCLK:\"");
    if (idx < 0) {
        Serial.println("[NITZ] Sin respuesta +CCLK");
        if (g_comm_mode == 2) {
            SerialGSM.println("AT+CSCLK=2"); vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(GSM_SLEEP_PIN, HIGH);
            Serial.println("[NITZ] Modem a sleep (SD-only, sin +CCLK)");
        }
        return false;
    }

    String dt = resp.substring(idx + 8);
    dt.trim();

    int yy  = dt.substring(0, 2).toInt() + 2000;
    int mon = dt.substring(3, 5).toInt();
    int dd  = dt.substring(6, 8).toInt();
    int hh  = dt.substring(9, 11).toInt();
    int mm  = dt.substring(12, 14).toInt();
    int ss  = dt.substring(15, 17).toInt();

    if (yy < 2024 || mon < 1 || mon > 12 || dd < 1 || dd > 31 ||
        hh > 23   || mm > 59 || ss > 59) {
        Serial.printf("[NITZ] Tiempo invalido: %04d-%02d-%02d %02d:%02d:%02d\n",
                      yy, mon, dd, hh, mm, ss);
        if (g_comm_mode == 2) {
            SerialGSM.println("AT+CSCLK=2"); vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(GSM_SLEEP_PIN, HIGH);
            Serial.println("[NITZ] Modem a sleep (SD-only, tiempo invalido)");
        }
        return false;
    }

    
    int tz_offset_s = 0;
    int tz_plus  = dt.indexOf('+', 17);
    int tz_minus = dt.indexOf('-', 17);
    int tz_idx   = -1;
    bool tz_neg  = false;
    if (tz_plus  >= 17) { tz_idx = tz_plus;  tz_neg = false; }
    if (tz_minus >= 17 && (tz_idx < 0 || tz_minus < tz_idx)) { tz_idx = tz_minus; tz_neg = true; }
    if (tz_idx >= 0) {
        int tz_quarters = dt.substring(tz_idx + 1).toInt();
        tz_offset_s = tz_quarters * 15 * 60;
        if (tz_neg) tz_offset_s = -tz_offset_s;
    }

    DateTime nitz_local(yy, mon, dd, hh, mm, ss);
    uint32_t unix_utc = (uint32_t)((int32_t)nitz_local.unixtime() - tz_offset_s);
    DateTime utc_time(unix_utc);
    rtc.adjust(utc_time);

    
    g_last_ntp_sync    = unix_utc;
    g_ts_epoch_ms      = (uint64_t)unix_utc * 1000ULL;
    g_ts_millis_anchor = millis();

    Serial.printf("[NITZ] Sync OK — UTC: %04d-%02d-%02d %02d:%02d:%02d (TZ offset %+ds)\n",
                  utc_time.year(), utc_time.month(), utc_time.day(),
                  utc_time.hour(), utc_time.minute(), utc_time.second(),
                  tz_offset_s);

    
    if (g_comm_mode == 2) {
        vTaskDelay(pdMS_TO_TICKS(200));
        SerialGSM.println("AT+CSCLK=2"); vTaskDelay(pdMS_TO_TICKS(300));
        digitalWrite(GSM_SLEEP_PIN, HIGH);
        Serial.println("[NITZ] Modem a sleep (SD-only mode)");
    }

    return true;
}

static void checkNTPStaleness() {
    if (g_last_ntp_sync == 0) return;
    uint32_t now_ts = rtc.now().unixtime();
    if (now_ts > g_last_ntp_sync && (now_ts - g_last_ntp_sync) > NTP_WARN_INTERVAL_S) {
        Serial.printf("[COMM] WARN: RTC sin sincronizar NTP por %lu horas\n", (unsigned long)((now_ts - g_last_ntp_sync) / 3600UL));
    }
}


static bool readGPS() {
    SerialGSM.println("AT+CGNSSPWR=1"); vTaskDelay(pdMS_TO_TICKS(500));
    String pwrResp = ""; uint32_t t0 = millis();
    while (millis() - t0 < 800) { if (SerialGSM.available()) pwrResp += (char)SerialGSM.read(); }
    Serial.printf("[%s][GPS] CGNSSPWR resp: %s\n", getTimeStr(), pwrResp.c_str());
    vTaskDelay(pdMS_TO_TICKS(300)); while (SerialGSM.available()) SerialGSM.read();
    vTaskDelay(pdMS_TO_TICKS(100)); while (SerialGSM.available()) SerialGSM.read();
    SerialGSM.println("AT+CGNSSINFO"); vTaskDelay(pdMS_TO_TICKS(1200));
    String resp = ""; t0 = millis();
    while (millis() - t0 < 1500) { if (SerialGSM.available()) resp += (char)SerialGSM.read(); }
    resp.trim();
    int idx = resp.indexOf("+CGNSSINFO:");
    if (idx < 0) { Serial.printf("[%s][GPS] ERROR: sin respuesta +CGNSSINFO\n", getTimeStr()); return false; }
    String data = resp.substring(idx + 12); data.trim();
    float fields[13] = {0};
    String token = "", latStr = "", lonStr = "", nsStr = "", ewStr = "";
    int pos = 0, fld = 0;
    while (pos <= (int)data.length() && fld < 13) {
        char c = (pos < (int)data.length()) ? data[pos] : ',';
        if (c == ',') {
            switch(fld) {
                case 0: if (token.toInt() == 0) { Serial.printf("[%s][GPS] Sin fix satelital\n", getTimeStr()); return false; } break;
                case 1: latStr = token; break; case 2: nsStr  = token; break;
                case 3: lonStr = token; break; case 4: ewStr  = token; break;
                case 11: fields[11] = token.toFloat(); break;
            }
            token = ""; fld++;
        } else token += c;
        pos++;
    }
    if (latStr.length() < 4 || lonStr.length() < 5) { Serial.printf("[%s][GPS] ERROR: formato lat/lon invalido\n", getTimeStr()); return false; }
    float latDeg = latStr.substring(0,2).toFloat(), latMin = latStr.substring(2).toFloat();
    float lat = latDeg + latMin / 60.0f; if (nsStr == "S") lat = -lat;
    float lonDeg = lonStr.substring(0,3).toFloat(), lonMin = lonStr.substring(3).toFloat();
    float lon = lonDeg + lonMin / 60.0f; if (ewStr == "W") lon = -lon;
    g_gps_lat = lat; g_gps_lon = lon; g_gps_hdop = fields[11]; g_gps_fix = 1;
    Serial.printf("[%s][GPS] Fix OK — Lat=%.6f Lon=%.6f HDOP=%.1f\n", getTimeStr(), lat, lon, fields[11]);
    return true;
}


// Appends a record to PENDING.csv for deferred upload on reconnect.
static void pendingEnqueue(const IEC_150_180_Record& r) {
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) != pdTRUE) return;
    digitalWrite(ISOLATOR_EN_PIN, LOW);
    delayMicroseconds(500);  
    File f = SD.open(PENDING_FILE, FILE_APPEND);
    if (f) {
        char line[1024];
        bool is_1p2w = (g_system_mode == SYS_MODE_1P2W);
        
        char _ph = g_evt_interruption ? g_evt_int_phase :
                   g_evt_sag         ? g_evt_sag_phase  :
                   g_evt_swell       ? g_evt_swl_phase  : '-';
        char phase_str[2] = {_ph, '\0'};
        snprintf(line, sizeof(line),
            "%llu|{\"ts\":%llu,"   
            "\"Vrms_A\":%.3f,\"Vrms_B\":%.3f,\"Vrms_C\":%.3f,"
            "\"freq_A\":%.3f,\"V_unb\":%.2f,"
            "\"Irms_A\":%.3f,\"Irms_N\":%.3f,"
            "\"Pa_A\":%.2f,\"Pa_B\":%.2f,\"Pa_C\":%.2f,"
            "\"Pr_A\":%.2f,\"Pr_B\":%.2f,\"Pr_C\":%.2f,"
            "\"Ps_A\":%.2f,\"Ps_B\":%.2f,\"Ps_C\":%.2f,"
            "\"PF_A\":%.3f,\"PF_B\":%.3f,\"PF_C\":%.3f,"
            "\"THDv_A\":%.2f,\"THDv_B\":%.2f,\"THDv_C\":%.2f,"
            "\"evt_sag\":%d,\"evt_swell\":%d,\"evt_interruption\":%d,"
            "\"evt_phase\":\"%s\",\"evt_duration_ms\":%u,"
            "\"oc_flag\":%d,"
            "\"comm_mode\":%d,\"failover_active\":%d,\"source\":\"fifo\","
            "\"temp_ade\":%.1f,\"batt_v\":%.2f,\"batt_pct\":%u}",
            (unsigned long long)r.timestamp,
            (unsigned long long)r.timestamp,
            r.vrms_a, is_1p2w ? 0.0f : r.vrms_b, is_1p2w ? 0.0f : r.vrms_c,
            r.freq_a, is_1p2w ? 0.0f : r.u2_voltage,
            r.irms_a, r.irms_n,
            r.pa_a,   is_1p2w ? 0.0f : r.pa_b,   is_1p2w ? 0.0f : r.pa_c,
            r.pr_a,   is_1p2w ? 0.0f : r.pr_b,   is_1p2w ? 0.0f : r.pr_c,
            r.papp_a, is_1p2w ? 0.0f : r.papp_b, is_1p2w ? 0.0f : r.papp_c,
            r.pf_a,   is_1p2w ? 0.0f : r.pf_b,   is_1p2w ? 0.0f : r.pf_c,
            r.thd_v_a, is_1p2w ? 0.0f : r.thd_v_b, is_1p2w ? 0.0f : r.thd_v_c,
            (int)(g_evt_sag?1:0), (int)(g_evt_swell?1:0),
            (int)(g_evt_interruption?1:0),
            phase_str,
            g_evt_interruption ? g_evt_int_dur :
            g_evt_sag          ? g_evt_sag_dur :
            g_evt_swell        ? g_evt_swl_dur : 0,
            (int)g_oc_flag,
            (int)(g_failover_active ? 1 : g_comm_mode), (int)(g_failover_active ? 1 : 0),
            r.temp_chip, g_batt_voltage, (unsigned)g_batt_percent);
        f.println(line); f.close();
        Serial.println("[PENDING] Record queued in PENDING.csv");
    }
    digitalWrite(ISOLATOR_EN_PIN, HIGH);
    xSemaphoreGive(xIsolatorMutex);
}

static int pendingCount() {
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(200)) != pdTRUE) return 0;
    digitalWrite(ISOLATOR_EN_PIN, LOW);
    delayMicroseconds(500);  
    int count = 0;
    File f = SD.open(PENDING_FILE, FILE_READ);
    if (f) { while (f.available()) { String line = f.readStringUntil('\n'); if (line.length() > 10) count++; } f.close(); }
    digitalWrite(ISOLATOR_EN_PIN, HIGH);
    xSemaphoreGive(xIsolatorMutex);
    return count;
}

static bool pendingSendOne(std::function<bool(const char*)> pubFn) {
    String json = "", line = "";
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
    digitalWrite(ISOLATOR_EN_PIN, LOW);
    delayMicroseconds(500);  
    File f = SD.open(PENDING_FILE, FILE_READ);
    if (!f || f.size() == 0) {
        if (f) f.close(); SD.remove(PENDING_FILE);
        digitalWrite(ISOLATOR_EN_PIN, HIGH); xSemaphoreGive(xIsolatorMutex); return false;
    }
    while (f.available()) { line = f.readStringUntil('\n'); line.trim(); if (line.length() > 10) break; line = ""; }
    String remaining = "";
    while (f.available()) { String l = f.readStringUntil('\n'); l.trim(); if (l.length() > 10) remaining += l + "\n"; }
    f.close();
    if (line.length() == 0) { SD.remove(PENDING_FILE); digitalWrite(ISOLATOR_EN_PIN, HIGH); xSemaphoreGive(xIsolatorMutex); return false; }
    SD.remove(PENDING_FILE);
    if (remaining.length() > 0) { File fw = SD.open(PENDING_FILE, FILE_WRITE); if (fw) { fw.print(remaining); fw.close(); } }
    digitalWrite(ISOLATOR_EN_PIN, HIGH); xSemaphoreGive(xIsolatorMutex);
    int sep = line.indexOf('|');
    json = (sep >= 0) ? line.substring(sep + 1) : line;
    bool ok = pubFn(json.c_str());
    if (!ok) {
        if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            digitalWrite(ISOLATOR_EN_PIN, LOW);
            delayMicroseconds(500);  
            String current = ""; File fc = SD.open(PENDING_FILE, FILE_READ);
            if (fc) { while (fc.available()) current += (char)fc.read(); fc.close(); }
            SD.remove(PENDING_FILE);
            File fw = SD.open(PENDING_FILE, FILE_WRITE);
            if (fw) { fw.println(line); fw.print(current); fw.close(); }
            digitalWrite(ISOLATOR_EN_PIN, HIGH); xSemaphoreGive(xIsolatorMutex);
        }
        Serial.println("[PENDING] Send failed — record returned to queue");
        return false;
    }
    Serial.println("[PENDING] Record sent OK");
    return true;
}


static bool publishEventAlarm(const IEC_Event& evt) {
    const char* type_str;
    int type_num;
    switch (evt.type) {
        case EVT_SAG:          type_str = "SAG";          type_num = 1; break;
        case EVT_SWELL:        type_str = "SWELL";        type_num = 2; break;
        case EVT_INTERRUPTION: type_str = "INTERRUPTION"; type_num = 3; break;
        default:               type_str = "UNKNOWN";      type_num = 0; break;
    }
    char phase_ch  = 'A' + evt.phase;
    char phase_str[2] = {phase_ch, '\0'};
    uint32_t duration_ms = evt.duration_ms;

    if (g_failover_active || g_comm_mode == 1) {
        if (!tb.connected()) return false;
        Telemetry alarm_data[] = {
            Telemetry("evt_alarm_type",       type_str),
            Telemetry("evt_alarm_type_num",   type_num),
            Telemetry("evt_alarm_phase",      phase_str),
            Telemetry("evt_alarm_ts_start",   (int)evt.timestamp_start),
            Telemetry("evt_alarm_ts_end",     (int)evt.timestamp_end),
            Telemetry("evt_alarm_residual_v", evt.residual_voltage),
            Telemetry("evt_alarm_peak_v",     evt.peak_voltage),
            Telemetry("evt_alarm_duration_ms",(int)duration_ms),
        };
        bool ok = tb.sendTelemetry<8>(std::begin(alarm_data), std::end(alarm_data));
        if (ok) Serial.printf("[ALARM] %s Fase %c via 4G\n", type_str, phase_ch);
        return ok;
    } else {
        if (!mqttActive || !mqttActive->connected()) return false;
        StaticJsonDocument<512> doc;
        doc["evt_alarm_type"]        = type_str;
        doc["evt_alarm_type_num"]    = type_num;
        doc["evt_alarm_phase"]       = phase_str;
        doc["evt_alarm_ts_start"]    = evt.timestamp_start;
        doc["evt_alarm_ts_end"]      = evt.timestamp_end;
        doc["evt_alarm_residual_v"]  = evt.residual_voltage;
        doc["evt_alarm_peak_v"]      = evt.peak_voltage;
        doc["evt_alarm_duration_ms"] = duration_ms;
        char buf[512];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        bool ok = mqttActive->publish(TB_TOPIC, buf, false);
        if (ok) Serial.printf("[ALARM] %s Fase %c via WiFi\n", type_str, phase_ch);
        return ok;
    }
}


// Writes a 3-second record to the daily CSV file on SD (SD-only mode).
static void saveToSD(const IEC_150_180_Record& r) {
    if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.println("[SD] No se pudo tomar mutex para CSV");
        return;
    }
    digitalWrite(ISOLATOR_EN_PIN, LOW);
    delayMicroseconds(500);  

    char csvName[32];
    uint32_t ts_sec = (uint32_t)(r.timestamp / 1000ULL);  
    time_t now_t = (time_t)ts_sec;
    const uint32_t TS_MIN_SD = 1704067200UL;
    const uint32_t TS_MAX_SD = 2051222400UL;
    if (ts_sec < TS_MIN_SD || ts_sec > TS_MAX_SD) {
        DateTime rtcNow = rtc.now();
        snprintf(csvName, sizeof(csvName), "/ITA9000_%04d%02d%02d.csv",
                 rtcNow.year(), rtcNow.month(), rtcNow.day());
    } else {
        struct tm* t = localtime(&now_t);
        if (t == nullptr) {
            DateTime rtcNow = rtc.now();
            snprintf(csvName, sizeof(csvName), "/ITA9000_%04d%02d%02d.csv",
                     rtcNow.year(), rtcNow.month(), rtcNow.day());
        } else {
            snprintf(csvName, sizeof(csvName), "/ITA9000_%04d%02d%02d.csv",
                     t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
        }
    }

    File f = SD.open(csvName, FILE_APPEND);
    if (!f) {
        digitalWrite(ISOLATOR_EN_PIN, HIGH);
        xSemaphoreGive(xIsolatorMutex);
        Serial.println("[SD] Error abriendo CSV");
        return;
    }

    if (f.size() == 0) {
        f.println("ts,"
                  "Vrms_A,Vrms_B,Vrms_C,Vf_A,Vf_B,Vf_C,V_unb,"
                  "Irms_A,Irms_B,Irms_C,Irms_N,If_A,If_B,If_C,I_unb,"
                  "Pa_A,Pa_B,Pa_C,Pr_A,Pr_B,Pr_C,Ps_A,Ps_B,Ps_C,"
                  "Paf_A,Paf_B,Paf_C,Prf_A,Prf_B,Prf_C,Psf_A,Psf_B,Psf_C,"
                  "PF_A,PF_B,PF_C,freq_A,freq_B,freq_C,"
                  "THDv_A,THDv_B,THDv_C,THDi_A,THDi_B,THDi_C,"
                  "ang_AB,ang_BC,ang_AC,ang_A,ang_B,ang_C,"
                  "ang_IA_IB,ang_IB_IC,ang_IA_IC,"
                  "Vh_A,Vh_B,Vh_C,"
                  "vpeak,ipeak,oc_flag,oc_phase,oc_peak_a,oc_peak_b,oc_peak_c,phase_seq,"
                  "gps_lat,gps_lon,gps_fix,gps_hdop,"
                  "evt_sag,evt_swell,evt_phase,evt_mag,evt_dur,"
                  "temp_ade,batt_v,batt_pct");
        f.flush();
    }

    IEC_150_180_Record m = r;
    bool is_1p2w = (g_system_mode == SYS_MODE_1P2W);
    bool is_3p3w = (g_system_mode == SYS_MODE_3P3W);
    if (is_1p2w) {
        m.vrms_b = 0; m.vrms_c = 0; m.irms_b = 0; m.irms_c = 0;
        m.pa_b = 0; m.pa_c = 0; m.pr_b = 0; m.pr_c = 0;
        m.papp_b = 0; m.papp_c = 0;
        m.pa_fund_b = 0; m.pa_fund_c = 0;
        m.pr_fund_a = 0; m.pr_fund_b = 0; m.pr_fund_c = 0;
        m.papp_fund_b = 0; m.papp_fund_c = 0;
        m.vrms_fund_b = 0; m.vrms_fund_c = 0;
        m.irms_fund_b = 0; m.irms_fund_c = 0;
        m.pf_b = 0; m.pf_c = 0;
        m.thd_v_b = 0; m.thd_v_c = 0;
        m.thd_i_b = 0; m.thd_i_c = 0;
        m.vrms_half_b = 0; m.vrms_half_c = 0;
        m.angle_va_vb = 0; m.angle_vb_vc = 0; m.angle_va_vc = 0;
        m.angle_vb_ib = 0; m.angle_vc_ic = 0;
        m.angle_ia_ib = 0; m.angle_ib_ic = 0; m.angle_ia_ic = 0;
        m.u2_voltage = 0; m.u2_current = 0;
        m.freq_b = 0; m.freq_c = 0;
    }
    if (is_3p3w) m.irms_n = 0;

    bool ok = (f.printf(
        "%llu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%d,%d,%.2f,%.2f,%.2f,%d,"
        "%.6f,%.6f,%d,%.1f,"
        "%d,%d,%c,%.2f,%u,"
        "%.1f,%.2f,%d\n",
        m.timestamp,
        m.vrms_a, m.vrms_b, m.vrms_c,
        m.vrms_fund_a, m.vrms_fund_b, m.vrms_fund_c, m.u2_voltage,
        m.irms_a, m.irms_b, m.irms_c, m.irms_n,
        m.irms_fund_a, m.irms_fund_b, m.irms_fund_c, m.u2_current,
        m.pa_a, m.pa_b, m.pa_c, m.pr_a, m.pr_b, m.pr_c,
        m.papp_a, m.papp_b, m.papp_c,
        m.pa_fund_a, m.pa_fund_b, m.pa_fund_c,
        m.pr_fund_a, m.pr_fund_b, m.pr_fund_c,
        m.papp_fund_a, m.papp_fund_b, m.papp_fund_c,
        m.pf_a, m.pf_b, m.pf_c,
        m.freq_a, m.freq_b, m.freq_c,
        m.thd_v_a, m.thd_v_b, m.thd_v_c,
        m.thd_i_a, m.thd_i_b, m.thd_i_c,
        m.angle_va_vb, m.angle_vb_vc, m.angle_va_vc,
        m.angle_va_ia, m.angle_vb_ib, m.angle_vc_ic,
        m.angle_ia_ib, m.angle_ib_ic, m.angle_ia_ic,
        m.vrms_half_a, m.vrms_half_b, m.vrms_half_c,
        g_vpeak, g_ipeak, (int)g_oc_flag, (int)g_oc_phase,
        g_oc_peak_a, g_oc_peak_b, g_oc_peak_c, (int)g_phase_seq,
        g_gps_lat, g_gps_lon, (int)g_gps_fix, g_gps_hdop,
        g_evt_sag?1:0, g_evt_swell?1:0,
        (g_evt_interruption ? g_evt_int_phase :
         g_evt_sag          ? g_evt_sag_phase :
         g_evt_swell        ? g_evt_swl_phase : '-'),
        (g_evt_interruption ? g_evt_int_res :
         g_evt_sag          ? g_evt_sag_res  : 0.0f),
        (g_evt_interruption ? g_evt_int_dur :
         g_evt_sag          ? g_evt_sag_dur  :
         g_evt_swell        ? g_evt_swl_dur  : 0),
        m.temp_chip, g_batt_voltage, g_batt_percent) > 0);

    f.flush(); f.close();
    digitalWrite(ISOLATOR_EN_PIN, HIGH);
    xSemaphoreGive(xIsolatorMutex);
    if (ok) Serial.printf("[%s][SD] Fila guardada: %s\n", getTimeStr(), csvName);
    else    Serial.printf("[%s][SD] ERROR escritura: %s\n", getTimeStr(), csvName);
}


// Core 1 task: dual-path MQTT (WiFi primary, 4G fallback), PENDING flush, NTP/NITZ.
void taskComm(void* pvParameters) {
    Serial.println("[CORE1] taskComm iniciado.");

    wifiClientSecure.setInsecure();

    uint8_t  fail_count      = 0;
    bool     ntp_done        = false;
    uint32_t last_ntp_ms     = 0;
    bool     gsm_ready       = false;
    uint8_t  active_mode     = 255;
    bool     auto_failover   = false;
    uint32_t last_wifi_retry = 0;
    bool     prev_oc_flag    = false;

    vTaskDelay(pdMS_TO_TICKS(3000));

    int pending = pendingCount();
    if (pending > 0) {
        Serial.printf("[COMM] %d registros pendientes en PENDING.csv\n", pending);
    }

    while (true) {
        uint8_t mode = g_comm_mode;

        
        if (g_pending_flush_on_start) {
            g_pending_flush_on_start = false;
            Serial.println("[COMM] START flush — vaciando PENDING antes de nueva sesion...");
            bool flushed = false;
            if (active_mode == 0) {
                bool net_ok  = connectWiFi();
                bool mqtt_ok = net_ok && connectMQTT();
                if (mqtt_ok) {
                    int n = pendingCount();
                    while (pendingCount() > 0) {
                        bool sent = pendingSendOne([](const char* json) -> bool {
                            return mqttActive->publish(TB_TOPIC, json, false);
                        });
                        if (!sent) break;
                        mqttActive->loop();
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    flushed = (pendingCount() == 0);
                    Serial.printf("[COMM] START flush WiFi: %d registros enviados. %s\n",
                                  n, flushed ? "OK" : "parcial");
                }
            } else if (active_mode == 1) {
                if (!gsm_ready) gsm_ready = connect4G();
                if (gsm_ready && !tb.connected()) tb.connect(TB_SERVER, TB_TOKEN, TB_PORT_4G);
                if (tb.connected()) {
                    int n = pendingCount();
                    while (pendingCount() > 0) {
                        bool sent = pendingSendOne([](const char* json) -> bool {
                            return tb.sendTelemetryString(json);
                        });
                        if (!sent) break;
                        tb.loop();
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    flushed = (pendingCount() == 0);
                    Serial.printf("[COMM] START flush 4G: %d registros enviados. %s\n",
                                  n, flushed ? "OK" : "parcial");
                }
            }
            if (!flushed) {
                
                if (xSemaphoreTake(xIsolatorMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                    digitalWrite(ISOLATOR_EN_PIN, LOW);
                    delayMicroseconds(500);
                    SD.remove(PENDING_FILE);
                    digitalWrite(ISOLATOR_EN_PIN, HIGH);
                    xSemaphoreGive(xIsolatorMutex);
                    Serial.println("[COMM] START flush: sin conexion — PENDING.csv borrado.");
                }
            }
        }
        if (mode == 0 && fail_count >= WIFI_FAIL_THRESHOLD && !auto_failover) {
            auto_failover = true;
            active_mode   = 1;
            gsm_ready     = false;
            last_wifi_retry = millis();
            Serial.printf("[COMM] FAILOVER — WiFi fallo %d veces → 4G\n", fail_count);
            g_comm_state      = COMM_STATE_DOWN;
            g_failover_active = true;
            g_beep_request    = BEEP_MODE_CHANGE;
        }

        if (auto_failover && mode != 0) auto_failover = false;
        if (auto_failover) { active_mode = 1; }
        else {
            uint8_t prev_mode = active_mode;
            active_mode = mode;
            if (prev_mode != mode) { fail_count = 0; Serial.printf("[COMM] Modo cambiado a %d — reset fail_count\n", mode); }
        }

        
        if (auto_failover && (millis() - last_wifi_retry) >= WIFI_RECOVERY_INTERVAL_MS) {
            last_wifi_retry = millis();
            Serial.println("[COMM] Intentando recuperacion WiFi...");
            if (connectWiFi()) {
                Serial.println("[COMM] WiFi recuperado — volviendo a WiFi");
                auto_failover = false; active_mode = 0; fail_count = 0;
                g_comm_state = COMM_STATE_OK; g_failover_active = false;
                g_beep_request = BEEP_MODE_CHANGE;
                g_alarm_failover_sent = false;
                syncNTP(); ntp_done = true; last_ntp_ms = millis();
            } else {
                Serial.println("[COMM] WiFi no disponible — continuando en 4G");
            }
        }

        
        {
            uint32_t now_ms = millis();
            bool ntp_needed = !ntp_done ||
                              (now_ms - last_ntp_ms) >= NTP_REFRESH_INTERVAL_MS;
            if (ntp_needed) {
                if (active_mode == 0 && connectWiFi()) {
                    syncNTP();
                    ntp_done = true; last_ntp_ms = now_ms;
                } else if (active_mode == 1) {
                    if (syncNTP_4G()) {
                        ntp_done = true; last_ntp_ms = now_ms;
                    } else if (syncNITZ()) {
                        ntp_done = true; last_ntp_ms = now_ms;
                    }
                } else if (active_mode == 2) {
                    if (syncNITZ()) {
                        ntp_done = true; last_ntp_ms = now_ms;
                    }
                }
            }
            checkNTPStaleness();
        }

        
        IEC_150_180_Record rec;
        if (xQueueReceive(xQueue150Comm, &rec, pdMS_TO_TICKS(5000)) != pdTRUE) {
            if (active_mode == 0 && mqttActive->connected()) mqttActive->loop();
            if (active_mode == 1 && tb.connected()) tb.loop();
            continue;
        }
        if (!g_measuring) continue;

        
        if (active_mode != 2) {
            uint32_t now_ms  = millis();
            bool gps_due = ((now_ms - g_gps_last_ms) >= GPS_PUBLISH_INTERVAL_MS);
            if (gps_due) {
                if (active_mode == 1 && tb.connected()) { tb.disconnect(); vTaskDelay(pdMS_TO_TICKS(100)); }
                readGPS();
                if (active_mode == 1) {
                    if (connect4G() && tb.connect(TB_SERVER, TB_TOKEN, TB_PORT_4G)) publishGPS(true);
                } else {
                    publishGPS(false);
                }
                g_gps_last_ms = now_ms;
            }
        }

        
        if (active_mode != 2) {
            IEC_Event pending_evt;
            if (xQueueReceive(xQueueEventComm, &pending_evt, 0) == pdTRUE) {
                publishEventAlarm(pending_evt);
            }
            if (g_oc_flag && !prev_oc_flag) publishOCAlarm(active_mode == 1);
            prev_oc_flag = (g_oc_flag != 0);
        }

        
        if (!g_start_attrs_sent && active_mode != 2) {
            publishStartAttributes(active_mode == 1);
        }

        
        if (active_mode == 2) {
            g_comm_state = COMM_STATE_SD_ONLY;
            saveToSD(rec);
            g_comm_success = true;
            continue;
        }

        
        if (active_mode == 0) {
            mqttActive  = &mqttWifi;
            bool net_ok  = connectWiFi();
            bool mqtt_ok = net_ok && connectMQTT();
            bool pub_ok  = false;

            if (mqtt_ok) {
                while (pendingCount() > 0) {
                    bool sent = pendingSendOne([](const char* json) -> bool {
                        return mqttActive->publish(TB_TOPIC, json, false);
                    });
                    if (!sent) break;
                    mqttActive->loop();
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                pub_ok = publishTelemetry(rec, false);  
                if (pub_ok) {
                    fail_count = 0; g_comm_state = COMM_STATE_OK; g_comm_success = true;

                
                if (g_batt_voltage > 0.5f && g_batt_voltage <= BATT_LOW_VOLT && !g_alarm_batt_sent) {
                    g_alarm_batt_sent = true;
                    StaticJsonDocument<256> alrm;
                    alrm["alarm_type"] = "LOW_BATT";
                    alrm["batt_v"]     = roundf(g_batt_voltage * 100.0f) / 100.0f;
                    alrm["batt_pct"]   = (int)g_batt_percent;
                    alrm["ts"]         = (uint64_t)getUnixTS() * 1000ULL;
                    publishRealAlarm("LOW_BATT", alrm, false);
                } else if (g_batt_voltage >= BATT_LOW_VOLT_RESET && g_alarm_batt_sent) {
                    g_alarm_batt_sent = false;
                }
                if (rec.temp_chip > 70.0f && !g_alarm_temp_sent) {
                    g_alarm_temp_sent = true;
                    StaticJsonDocument<256> alrm;
                    alrm["alarm_type"] = "HIGH_TEMP";
                    alrm["temp_chip"]  = roundf(rec.temp_chip * 10.0f) / 10.0f;
                    alrm["ts"]         = (uint64_t)getUnixTS() * 1000ULL;
                    publishRealAlarm("HIGH_TEMP", alrm, false);
                } else if (rec.temp_chip <= 65.0f && g_alarm_temp_sent) {
                    g_alarm_temp_sent = false;
                }
                } else {
                    fail_count++;
                    g_comm_state = (fail_count >= WIFI_FAIL_THRESHOLD) ? COMM_STATE_DOWN : COMM_STATE_WARN;
                    if (fail_count == 1) g_beep_request = BEEP_ERROR;
                    Serial.printf("[WIFI] Publish fallo — fallos: %d/%d\n", fail_count, WIFI_FAIL_THRESHOLD);
                    pendingEnqueue(rec);
                }
                mqttActive->loop();
            } else {
                fail_count++;
                g_comm_state = (fail_count >= WIFI_FAIL_THRESHOLD) ? COMM_STATE_DOWN : COMM_STATE_WARN;
                if (fail_count == 1) g_beep_request = BEEP_ERROR;
                Serial.printf("[WIFI] Conexion fallo — fallos: %d/%d\n", fail_count, WIFI_FAIL_THRESHOLD);
                pendingEnqueue(rec);
                vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
            }
            continue;
        }

        
        if (active_mode == 1) {
            if (!gsm_ready) {
                gsm_ready = connect4G();
                if (!gsm_ready) { g_comm_state = COMM_STATE_DOWN; pendingEnqueue(rec); vTaskDelay(pdMS_TO_TICKS(10000)); continue; }
            }
            if (!modem.isGprsConnected()) {
                gsm_ready = modem.gprsConnect(GSM_APN);
                if (!gsm_ready) { pendingEnqueue(rec); vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
            }
            if (!tb.connected()) {
                Serial.println("[4G] Conectando ThingsBoard...");
                static uint8_t tb_fail_count = 0;
                if (!tb.connect(TB_SERVER, TB_TOKEN, TB_PORT_4G)) {
                    g_comm_state = COMM_STATE_DOWN;
                    Serial.println("[4G] TB connect fallo");
                    pendingEnqueue(rec);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    tb_fail_count++;
                    if (tb_fail_count >= 5) {
                        tb_fail_count = 0;
                        Serial.println("[4G] TB backoff 30s tras 5 fallos consecutivos");
                        vTaskDelay(pdMS_TO_TICKS(30000));
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_INTERVAL_MS));
                    }
                    continue;
                }
                tb_fail_count = 0;
                Serial.println("[4G] ThingsBoard OK");
                while (pendingCount() > 0) {
                    bool sent = pendingSendOne([](const char* json) -> bool {
                        return tb.sendTelemetryString(json);
                    });
                    if (!sent) break;
                    tb.loop();
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
            }
            if (publishTelemetry4G(rec, false)) {  
                fail_count = 0; g_comm_state = COMM_STATE_OK; g_comm_success = true;

                
                if (g_batt_voltage > 0.5f && g_batt_voltage <= BATT_LOW_VOLT && !g_alarm_batt_sent) {
                    g_alarm_batt_sent = true;
                    StaticJsonDocument<256> alrm;
                    alrm["alarm_type"] = "LOW_BATT";
                    alrm["batt_v"]     = roundf(g_batt_voltage * 100.0f) / 100.0f;
                    alrm["batt_pct"]   = (int)g_batt_percent;
                    alrm["ts"]         = (uint64_t)getUnixTS() * 1000ULL;
                    publishRealAlarm("LOW_BATT", alrm, true);
                } else if (g_batt_voltage >= BATT_LOW_VOLT_RESET && g_alarm_batt_sent) {
                    g_alarm_batt_sent = false;
                }
                if (rec.temp_chip > 70.0f && !g_alarm_temp_sent) {
                    g_alarm_temp_sent = true;
                    StaticJsonDocument<256> alrm;
                    alrm["alarm_type"] = "HIGH_TEMP";
                    alrm["temp_chip"]  = roundf(rec.temp_chip * 10.0f) / 10.0f;
                    alrm["ts"]         = (uint64_t)getUnixTS() * 1000ULL;
                    publishRealAlarm("HIGH_TEMP", alrm, true);
                } else if (rec.temp_chip <= 65.0f && g_alarm_temp_sent) {
                    g_alarm_temp_sent = false;
                }
                
                if (auto_failover && !g_alarm_failover_sent) {
                    g_alarm_failover_sent = true;
                    StaticJsonDocument<256> alrm;
                    alrm["alarm_type"]      = "FAILOVER_ON";
                    alrm["failover_active"] = 1;
                    alrm["ts"]              = (uint64_t)getUnixTS() * 1000ULL;
                    publishRealAlarm("FAILOVER_ON", alrm, true);
                }
            } else {
                fail_count++; g_comm_state = COMM_STATE_DOWN;
                pendingEnqueue(rec);
            }
            tb.loop();
            continue;
        }
    }
}

// ---- Arduino entry points ----
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=====================================================");
    Serial.println("ITA-9000 PQM v25.8 — FreeRTOS + IEC + SD + Dual-Path");
    Serial.println("=====================================================");

    
    pinMode(PM_1_PIN,          OUTPUT); digitalWrite(PM_1_PIN, LOW);
    pinMode(ADE9000_RESET_PIN, OUTPUT); digitalWrite(ADE9000_RESET_PIN, HIGH);
    pinMode(ISOLATOR_EN_PIN,   OUTPUT); digitalWrite(ISOLATOR_EN_PIN, LOW);
    pinMode(SD_CS_PIN,         OUTPUT); digitalWrite(SD_CS_PIN, HIGH);
    pinMode(ADE9000_CS_PIN,    OUTPUT); digitalWrite(ADE9000_CS_PIN, HIGH);
    pinMode(IRQ0_PIN,          INPUT);
    pinMode(IRQ1_PIN,          INPUT);
    pinMode(DREADY_PIN,        INPUT);
    pinMode(RELAY_PIN,         OUTPUT); digitalWrite(RELAY_PIN, RELAY_ON);
    pinMode(BUZZER_PIN,        OUTPUT); digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[SETUP] Relay ON (controlled by HMI START/STOP).");

    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.print("[SETUP] Starting RTC DS3231... ");
    if (!rtc.begin()) {
        Serial.println("ERROR — RTC not found. Continuing without RTC.");
    } else {
        if (rtc.lostPower()) {
            Serial.println("RTC lost power — adjusting to compile time.");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        DateTime now = rtc.now();
        Serial.printf("OK. Time: %02d/%02d/%04d %02d:%02d:%02d\n",
                      now.day(), now.month(), now.year(),
                      now.hour(), now.minute(), now.second());
    }

    
    pinMode(INT_U3_PIN, INPUT);
    if (!pcfU2.begin() || !pcfU3.begin()) {
        Serial.println("[SETUP] ERROR: PCF8574 not found. HMI disabled.");
    } else {
        u2_state = 0xFF; u3_state = 0xFF;
        bitWrite(u3_state, LED_POWER_ON, LED_ON);
        pcfU2.write8(u2_state);
        pcfU3.write8(u3_state | 0x0F);
        delay(5000);   
        attachInterrupt(digitalPinToInterrupt(INT_U3_PIN), []() IRAM_ATTR {
            g_button_pressed = true;
        }, FALLING);
        Serial.println("[SETUP] HMI PCF8574 OK.");
    }

    
    bq25896.begin();
    delay(100);
    if (!bq25896.isConnected()) {
        Serial.println("[SETUP] WARNING: BQ25896 not found. BMS disabled.");
    } else {
        Serial.println("[SETUP] BQ25896 OK.");
        
        bq25896.setREG_RST(true);
        delay(150);  
        Serial.println("[SETUP] BQ25896 REG_RST completado — registros en default.");
        bq25896.setSYS_MIN(3000);
        Serial.printf("[SETUP] BQ25896 SYS_MIN=%u mV (era 3500 mV default)\n", bq25896.getSYS_MIN());
        bq25896.setCONV_RATE(true);
        Serial.println("[SETUP] BQ25896 CONV_RATE=1 (continuo 1s)");
    }

    
    SPI.begin();
    Serial.print("[SETUP] Starting SD (CS=GPIO26)... ");
    if (!SD.begin(SD_CS_PIN)) Serial.println("ERROR — SD not found. Continuing without SD.");
    else                       Serial.println("OK.");

    
    Serial.println("[SETUP] Starting ADE9000...");
    digitalWrite(ISOLATOR_EN_PIN, HIGH);
    delay(50);
    digitalWrite(ADE9000_RESET_PIN, LOW);  delay(50);
    digitalWrite(ADE9000_RESET_PIN, HIGH); delay(1000);

    ade9000.SPI_Init(5000000, ADE9000_CS_PIN);
    SPI.endTransaction();

    SPI.beginTransaction(adeSettings);
    ade9000.SetupADE9000();
    SPI.endTransaction();

    SPI.beginTransaction(adeSettings);
    ade9000.SPI_Write_16(ADDR_RUN, 0x0000);
    SPI.endTransaction();
    delay(100);

    if (loadCalibrationFromEEPROM()) Serial.println("[SETUP] Calibration loaded from EEPROM.");
    else                              Serial.println("[SETUP] WARNING: Using factory calibration (no EEPROM).");

    SPI.beginTransaction(adeSettings);
    ade9000.SPI_Write_16(ADDR_ACCMODE, 0x3100);
    ade9000.SPI_Write_16(ADDR_ZX_LP_SEL, 0x0000);
    ade9000.SPI_Write_16(ADDR_CONFIG1, 0x000E);
    uint16_t config1_rb = ade9000.SPI_Read_16(ADDR_CONFIG1);
    Serial.printf("[SETUP] CONFIG1=0x%04X (expected 0x000E, DREADY enabled)\n", config1_rb);
    ade9000.SPI_Write_32(ADDR_EVENT_MASK, 0x00770000);
    ade9000.SPI_Write_16(ADDR_CONFIG3, 0x0007);
    ade9000.SPI_Write_32(ADDR_OILVL, 26607987UL);
    Serial.println("[SETUP] Overcurrent: 100 A threshold enabled on phases A/B/C.");
    ade9000.SPI_Write_32(ADDR_MASK0, 0x00100000UL);
    ade9000.SPI_Write_16(ADDR_RUN, 0x0001);
    delay(500);
    ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFFUL);
    ade9000.SPI_Write_32(ADDR_STATUS1, 0xFFFFFFFFUL);
    
    
    ade9000.SPI_Write_32(ADDR_MASK1, 0x03E20000UL);
    
    ade9000.SPI_Write_16(ADDR_DIP_CYC,   EVT_DIP_CYC);
    ade9000.SPI_Write_16(ADDR_SWELL_CYC, EVT_SWELL_CYC);
    uint16_t run_val = ade9000.SPI_Read_16(ADDR_RUN);
    SPI.endTransaction();
    Serial.printf("[SETUP] ADE9000 RUN=0x%04X (expected 0x0001)\n", run_val);
    if (run_val != 0x0001) {
        Serial.println("[SETUP] ERROR: ADE9000 not responding correctly!");
    } else {
        Serial.println("[SETUP] ADE9000 OK.");
    }

    
    attachInterrupt(digitalPinToInterrupt(DREADY_PIN), isr_dready, RISING);
    attachInterrupt(digitalPinToInterrupt(IRQ0_PIN),   isr_irq0,   FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ1_PIN),   isr_irq1,   FALLING);
    Serial.println("[SETUP] Interrupts configured: DREADY(GPIO34), IRQ0(GPIO39), IRQ1(GPIO36).");
    g_boot_ms = millis();

    
    xQueue10min     = xQueueCreate(5,  sizeof(IEC_10MIN_Record));
    xQueueEvents    = xQueueCreate(20, sizeof(IEC_Event));
    xQueue150Comm   = xQueueCreate(20, sizeof(IEC_150_180_Record));  
    xQueueEventComm = xQueueCreate(10, sizeof(IEC_Event));
    xIsolatorMutex  = xSemaphoreCreateMutex();

    if (!xQueue10min || !xQueueEvents || !xQueue150Comm || !xQueueEventComm || !xIsolatorMutex) {
        Serial.println("[SETUP] CRITICAL ERROR: Could not create queues/mutex. Insufficient heap?");
        while(1) delay(1000);
    }
    Serial.println("[SETUP] Queues and mutex created.");

    
    setupHardwareEventThresholds();
    Serial.println("[SETUP] Hardware DIP/SWELL thresholds configured.");

    
    pinMode(GSM_SLEEP_PIN, OUTPUT);
    digitalWrite(GSM_SLEEP_PIN, LOW);
    delay(500);
    SerialGSM.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(5000);
    SerialGSM.println("AT+CSCLK=0");
    delay(500);
    while (SerialGSM.available()) SerialGSM.read();
    Serial.println("[SETUP] Serial2 4G module initialized (CSCLK=0).");

    
    xTaskCreatePinnedToCore(taskADE9000,     "ADE9000",   12288, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(taskComm,        "Comm",      12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(taskSD,          "SD_Writer",  4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskHMI,         "HMI",        4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskBMS,         "BMS",        3072, NULL, 1, NULL, 1);

    Serial.println("[SETUP] FreeRTOS tasks created.");

    
    u2_state = 0x00; u3_state = 0x00;
    pcfU2.write8(u2_state);
    pcfU3.write8(u3_state | 0x0F);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(750);
    digitalWrite(BUZZER_PIN, LOW);
    u2_state = 0xFF; u3_state = 0xFF;
    bitWrite(u3_state, LED_POWER_ON, LED_ON);
    bitWrite(u2_state, LED_3P4W,     LED_ON);
    bitWrite(u2_state, LED_WIFI,     LED_ON);
    pcfU2.write8(u2_state);
    pcfU3.write8(u3_state | 0x0F);

    Serial.println("[SETUP] System ready. IEC pipeline starting...");
    Serial.println("=====================================================\n");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}

