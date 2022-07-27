/************************************************************************
 * LoraWAN EasyNode 
 * main.c
 * rev 1.0 - 6 july 2022 - shabaz
 * This project requires the pico-lorawan and no-OS-FatFS-SD-SPI-RPi-Pico
 * projects in order to build
 ************************************************************************/
 
// ********** header files *****************
#include <stdio.h>
#include <string.h>
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "pico/unique_id.h"
#include "tusb.h"
// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"
//
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "sd_card.h"

// ***************** defines ***************
// #define NVM_USAGE_CHECK
// power enable pin
#define VP_EN_PIN 22
// keep_awake pin
#define KEEP_AWAKE_PIN 15
// external button pin (INIT)
#define BUTTON_INIT_PIN 14
#define RFM_SPI_SS 8
//
#define LORA_STACK_NV_START ((PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * 1)))
#define NV_START ((PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * 2)))
#define NV_READ_ADDRESS ((const uint8_t*)(XIP_BASE + NV_START))
//
#define POWER_VP_ON gpio_put(VP_EN_PIN, 1)
#define POWER_VP_OFF gpio_put(VP_EN_PIN, 0)
#define PICO_LED_ON gpio_put(PICO_DEFAULT_LED_PIN, 1)
#define PICO_LED_OFF gpio_put(PICO_DEFAULT_LED_PIN, 0)
#define KEEP_AWAKE_ON gpio_put(KEEP_AWAKE_PIN, 1)
#define KEEP_AWAKE_OFF gpio_put(KEEP_AWAKE_PIN, 0)
#define INIT_UNPRESSED (gpio_get(BUTTON_INIT_PIN)!=0)
#define INIT_PRESSED (gpio_get(BUTTON_INIT_PIN)==0)
#define RFM_SPI_DIS gpio_put(RFM_SPI_SS, 1)
#define RFM_SPI_EN gpio_put(RFM_SPI_SS, 0)
#define PRINT_IF_FILE_ERROR(X) if (fr!=FR_OK) {printf(X);}

// MSP430 system power controller I2C
#define SYSPWR_ADDR 0x40
#define SYSPWR_WAKE_SEC 0x66
#define SYSPWR_WAKE_MIN 0x67
#define SYSPWR_INPUT 0x69
#define SYSPWR_OUTPUT 0x70
#define SYSPWR_TIME 0x64
#define SYSPWR_DATE 0x65



// MIC280-0YM6 thermal sensor I2C address is 0x48
#define THERM_ADDR 0x48
#define THERM_TEMP0 0x00
#define THERM_STATUS 0x02
#define THERM_CONFIG 0x03
#define THERM_IMASK 0x04
#define THERM_LOCK 0x09
#define THERM_REM_HIGH_MSB 0x07

// ******** constants ******************
const char* const config_item_name[] = {    "lorawan_region",           /* 0 */
                                            "lorawan_device_eui",       /* 1 */
                                            "lorawan_app_or_join_eui",  /* 2 */
                                            "lorawan_app_key",          /* 3 */
                                            "wake_after_sec",           /* 4 */
                                            "date",                     /* 5 */
                                            "month",                    /* 6 */
                                            "year",                     /* 7 */
                                            "time",                     /* 8 */
                                            "random_seed",              /* 9 */
                                            "-" /*END*/};

const int b64invs[] = { 62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58,
        59, 60, 61, -1, -1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
        21, 22, 23, 24, 25, -1, -1, -1, -1, -1, -1, 26, 27, 28,
        29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
        43, 44, 45, 46, 47, 48, 49, 50, 51 };

const char fields[]="YYYY-MM-DDTHH:MM:SS,YYY/MM/DD,HH:MM:SS,T1_CELSIUS\r\n";

// ************ global variables *********************
// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// OTAA settings
char s_device_eui[18];
char s_app_eui[18];
char s_app_key[34];

struct lorawan_otaa_settings otaa_settings; 
/* = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
}; 
*/



LoRaMacRegion_t lorawan_region;

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

// file handling
FATFS fs;
FIL fp;

int rtc_date;
int rtc_month;
int rtc_year;
char rtc_time[12];

uint16_t seed_arr[4];
uint16_t lfsr[4];

uint16_t wake_sec = 30; // wake interval

//char seed[]="SoIL1iE18b0="; // 4a82 0bd6 2135 f1bd
char seed[]="ESIzRFVmd4g=";

// ************* function prototypes ************
void internal_temperature_init();
float internal_temperature_get();
char rbyte(void);

// ********** functions *************************

void nv_erase(void)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase (NV_START, FLASH_SECTOR_SIZE * 1); // erase 4kB
    restore_interrupts(ints);
}

void lora_stack_nv_erase(void)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase (LORA_STACK_NV_START, FLASH_SECTOR_SIZE * 1); // erase 4kB
    restore_interrupts(ints);
}

void nv_write_256(char* buf)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program (NV_START, buf, 256); // write 256 bytes
    restore_interrupts(ints);
}

void nv_read_256(char* buf)
{
    int i;
    char *p = (char*)NV_READ_ADDRESS;  //(char *)(XIP_BASE+NV_START);
    for (i=0; i<256; i++) {
        buf[i] = p[i]; //*p;
    }
}

size_t b64_decoded_size(const char *in)
{
    size_t len;
    size_t ret;
    size_t i;

    if (in == NULL)
        return 0;

    len = strlen(in);
    ret = len / 4 * 3;

    for (i=len; i-->0; ) {
        if (in[i] == '=') {
            ret--;
        } else {
            break;
        }
    }

    return ret;
}

int b64_isvalidchar(char c)
{
    if (c >= '0' && c <= '9')
        return 1;
    if (c >= 'A' && c <= 'Z')
        return 1;
    if (c >= 'a' && c <= 'z')
        return 1;
    if (c == '+' || c == '/' || c == '=')
        return 1;
    return 0;
}

int b64_decode(const char *in, unsigned char *out, size_t outlen)
{
    size_t len;
    size_t i;
    size_t j;
    int    v;

    if (in == NULL || out == NULL)
        return 0;

    len = strlen(in);
    if (outlen < b64_decoded_size(in) || len % 4 != 0)
        return 0;

    for (i=0; i<len; i++) {
        if (!b64_isvalidchar(in[i])) {
            return 0;
        }
    }
    for (i=0, j=0; i<len; i+=4, j+=3) {
        v = b64invs[in[i]-43];
        v = (v << 6) | b64invs[in[i+1]-43];
        v = in[i+2]=='=' ? v << 6 : (v << 6) | b64invs[in[i+2]-43];
        v = in[i+3]=='=' ? v << 6 : (v << 6) | b64invs[in[i+3]-43];

        out[j] = (v >> 16) & 0xFF;
        if (in[i+2] != '=')
            out[j+1] = (v >> 8) & 0xFF;
        if (in[i+3] != '=')
            out[j+2] = v & 0xFF;
    }

    return 1;
}

void rseed(char* seed_encoded_text) {
    int i;
    int j = 0;
    uint16_t a, b;
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    b64_decode(seed_encoded_text, (char*)seed_arr, 8);
    printf("seed val is: 0x%04x %04x %04x %04x ", seed_arr[0], seed_arr[1], seed_arr[2], seed_arr[3]);

    // XOR the seed with the Flash unique ID
    for (i=0; i<4; i++) {
        a = (uint16_t)board_id.id[j++];
        b = (uint16_t)board_id.id[j++];
        a = (a<<8) | b;
        seed_arr[i] = seed_arr[i] ^ a;
    }
    printf("seed new is: 0x%04x %04x %04x %04x \n", seed_arr[0], seed_arr[1], seed_arr[2], seed_arr[3]);


    for (i=0; i<4; i++) {
        lfsr[i] = seed_arr[i];
    }

    // lets exercise the LFSR and start at a fixed offset from rseed
    for (i=0; i<8; i++) {
        rbyte();
    }

}

char rbyte(void)
{
    uint16_t bit[4];
    int c = 0;
    char res;

    do
    {
        bit[0] = ((lfsr[0] >> 0) ^ (lfsr[0] >> 1)  ^ (lfsr[0] >> 3)  ^ (lfsr[0] >> 4)) & 1u;
        lfsr[3] = lfsr[3]>>1 | (lfsr[2] << 15);
        lfsr[2] = lfsr[2]>>1 | (lfsr[1] << 15);
        lfsr[1] = lfsr[1]>>1 | (lfsr[0] << 15);
        lfsr[0] = lfsr[0]>>1 | (bit[0] << 15);
        c++;
    } while(c<8);

    res = (char)((lfsr[0]>>8) & 0xff);
    //printf("random byte: 0x%02x\n", res);
    return  (res);
}

// pico temperature functions
void internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}
float internal_temperature_get()
{
    const float v_ref = 3.3;

    // select and read the ADC
    adc_select_input(4);
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature, using the formula from 
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}

// MIC280 I2C Thermal Sensor functions
void
therm_init(void)
{
    uint8_t buf[] = {THERM_CONFIG, 0x00}; // starts a temperature conversion
    i2c_write_blocking(i2c_default, THERM_ADDR, buf, 2, false);
    buf[0] = THERM_REM_HIGH_MSB;
    buf[1] = 0x7f;
    i2c_write_blocking(i2c_default, THERM_ADDR, buf, 2, false);
    buf[0] = THERM_IMASK;
    buf[1] = 0x80;
    i2c_write_blocking(i2c_default, THERM_ADDR, buf, 2, false);
}

int
therm_is_ready(void)
{
    int retval = 0;
    uint8_t buf[] = {THERM_STATUS};
    i2c_write_blocking(i2c_default, THERM_ADDR, buf, 1, true);
    i2c_read_blocking(i2c_default, THERM_ADDR, buf, 1, false);
    if (buf[0] & 0x80) {
        retval = 1; // sensor is ready to provide a result
    }
    return(retval);
}

int
therm_value(void)
{
    uint8_t buf[] = {THERM_TEMP0};
    i2c_write_blocking(i2c_default, THERM_ADDR, buf, 1, true);
    i2c_read_blocking(i2c_default, THERM_ADDR, buf, 1, false);
    return((int)buf[0]);
}

int
therm_wait_get_value(int do_init)
{
    if (do_init) {
        therm_init();
    }
    while (therm_is_ready()==0) {
        sleep_ms(100);
    }
    return(therm_value());
}


int
syspwr_test(void)
{
    int retval = 0;
    uint8_t buf[3];
    // test the MSP chip can be written and read via I2C
    buf[0] = 0; // address 0
    buf[1] = 0x11;
    buf[2] = 0x22;
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 3, false);
    sleep_ms(10);
    buf[1] = 0x33;
    buf[2] = 0x44;
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 3, false);
    sleep_ms(10);
    buf[1] = 0;
    buf[2] = 0;
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 1, false);
    i2c_read_blocking(i2c_default, SYSPWR_ADDR, buf, 2, false);
    if ((buf[0]=0x33) && (buf[1]=0x44)) {
        printf("syspwr_test passed\n");
    } else {
        printf("syspwr_test failed!\n");
        retval = -1;
    }
    return(retval);
}

// time_string must be at least 9 bytes long to obtain the time in this format:
// 21:58:01
void
syspwr_get_time(char* time_string)
{
    uint8_t buf[4];
    buf[0] = SYSPWR_TIME;

    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 1, false);
    i2c_read_blocking(i2c_default, SYSPWR_ADDR, buf, 4, false);
    sprintf(time_string, "%02x:%02x:%02x", buf[0], buf[1], buf[2]);
    if (buf[3]==0) {
        // AM
    } else {
        // PM, so add 12 to the hours
        if (buf[1]<'8') {
            buf[1] = buf[1]+2;
            buf[0] = buf[0]+1;
        } else if (buf[1]=='8') {
            buf[1] = '0';
            buf[0] = buf[0]+2;
        } else if (buf[1]=='9') {
            buf[1] = '1';
            buf[0] = buf[0]+2;
        }
    }
    printf("syspwr_get_time read: %s\n", time_string);
}

// converts the bcd char into two ascii digits
void
bcd_to_string(char bcd, char* asciidigit)
{
   asciidigit[0] = ((bcd >> 4) & 0x0f) + '0';
   asciidigit[1] = (bcd & 0x0f) + '0';
}

// date_string must be at least 11 bytes long to obtain the date in this format:
// 2022-11-28
void
syspwr_get_date(char* date_string)
{
    uint8_t buf[4];
    char temp_string[3];

    buf[0] = SYSPWR_DATE;
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 1, false);
    i2c_read_blocking(i2c_default, SYSPWR_ADDR, buf, 3, false);
    printf("read: %d, %d, %d\n", buf[0], buf[1], buf[2]);
    // handle month which is 0x00-0x11
    if (buf[1]<0x09) {
        buf[1]++;
    } else if (buf[1]==0x09) {
        buf[1]=0x10;
    } else if (buf[1]>=0x10) {
        buf[1]++;
    }
    bcd_to_string(buf[1], &date_string[5]);

    // handle date which is 0x01-0x31
    bcd_to_string(buf[2], &date_string[8]);

    // handle year which is 0x00-0x99
    date_string[0]='2';
    date_string[1]='0';
    bcd_to_string(buf[0], &date_string[2]);
    //sprintf(&date_string[4], "-%02d-%02d", buf[1], buf[2]);

    // insert separators and end of string
    date_string[4]='-';
    date_string[7]='-';
    date_string[10]='\0';

    printf("syspwr_get_date string: %s\n", date_string);
}

void syspwr_set_outputs(char val)
{
    uint8_t buf[2];
    buf[0] = SYSPWR_OUTPUT;
    buf[1] = val & 0x03; // only interested in the two least significant bits (OUT0_MSP and OUT1_MSP)
    printf("[OUT0, OUT1] set to: ");
    switch (buf[1]) {
        case 0:
            printf("[Open Drain, Open Drain]\n");
            break;
        case 1:
            printf("[Open Drain, 0V]\n");
            break;
        case 2:
            printf("[0V, Open Drain]\n");
            break;
        case 3:
            printf("[0V, 0V]\n");
            break;
        default:
            break;
    }
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 2, false);
}

void
syspwr_set_wake_time(uint16_t ws)
{
    uint8_t buf[2];
    uint16_t wm = 0;
    if (ws>255) {
       wm = ws / 60;
       buf[0] = SYSPWR_WAKE_MIN;
       buf[1] = (uint8_t)wm;
       printf("setting a wake time of %d min\n", buf[1]);
    } else {
        buf[0] = SYSPWR_WAKE_SEC;
        buf[1] = (uint8_t)ws;
        printf("setting a wake time of %d sec\n", buf[1]);
    }
    i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 2, false);
}

void
board_init(void)
{
    // keep power awake pin for system power controller
    gpio_init(KEEP_AWAKE_PIN);
    gpio_set_dir(KEEP_AWAKE_PIN, GPIO_OUT);
    KEEP_AWAKE_ON;

    // LED on Pico board
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // power enable pin for LDO for LoRaWAN module
    gpio_init(VP_EN_PIN);
    gpio_set_dir(VP_EN_PIN, GPIO_OUT);
    // button for input (INIT button)
    gpio_init(BUTTON_INIT_PIN);
    gpio_set_dir(BUTTON_INIT_PIN, GPIO_IN);
    gpio_set_pulls(BUTTON_INIT_PIN, true, false); // pullup enabled

    gpio_init(RFM_SPI_SS);
    gpio_set_dir(RFM_SPI_SS, GPIO_OUT);

    i2c_init(i2c_default, 50000); // I2C0 on GPIO 4[SDA],5[SCL]
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN); // weak pull-ups but enable them anyway
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);



    adc_init();

}

void
read_config_file() {
    FRESULT fr;
    char line[128];
    char *token;
    char name_s[32];
    char value_s[70];
    int pair_found=0;
    char ** itemlist;
    char* oneitem;
    int i;
    int write_nv = 0;
    struct lorawan_otaa_settings *sp; 
    char buf[256];
    char seed_b64[16];
    char v = 0;
    char row = 0;
    char tlen;
    char rb;
    int do_date=0;
    int do_time=0;

    sp = &otaa_settings;
    itemlist = (char**)config_item_name;

    int not_finished = 1;
    fr = f_mount(&fs, "", 0); // mount the file system
    PRINT_IF_FILE_ERROR("file mount error!\n");
    fr = f_open(&fp, "config.txt", FA_READ);
    PRINT_IF_FILE_ERROR("error opening config.txt!\n");
    do {
        if (f_gets(line, sizeof line, &fp) == NULL) {
            not_finished = 0;
            printf("found end of config file\n");
        } else {
            row++;
            token = strtok(line, " ");
            if (token != NULL) {
                strcpy(name_s, token);
                token = strtok(NULL, " ");
                if (token != NULL) {
                    strcpy(value_s, token);
                    tlen = strlen(value_s);
                    if (value_s[tlen-2] <= 0x0d) {
                        value_s[tlen-2] = '\0';
                    } else if (value_s[tlen-1] <= 0x0d) {
                        value_s[tlen-1] = '\0';
                    }
                    pair_found = 1;
                }
                if (name_s[0]=='#') {
                    pair_found = 0;
                }
            }

        }
        if (pair_found) { // two items of text on a line. Now parse them.
            printf("decoding row %d '%s'\n", row, name_s);
            i = 0;
            oneitem = itemlist[i];
            do {
                if (strcmp(oneitem, name_s)==0) {
                    switch (i) {
                        case 0: // lorawan_region
                            if (strcmp("europe", (const char*) value_s) == 0) {
                                v = 0;
                                lorawan_region = LORAMAC_REGION_EU868;
                            }
                            if (strcmp("north_amer", (const char*) value_s) == 0) {
                                v = 1;
                                lorawan_region = LORAMAC_REGION_US915;
                            }
                            if (strcmp("aus", (const char*) value_s) == 0) {
                                v = 2;
                                lorawan_region = LORAMAC_REGION_AU915;
                            }
                            if (strcmp("india", (const char*) value_s) == 0) {
                                v = 3;
                                lorawan_region = LORAMAC_REGION_IN865;
                            }
                            write_nv = 1;
                            break;
                        case 1: // lorawan_device_eui
                            printf("found lorawan_device_eui\n");
                            strcpy(s_device_eui, (const char*)value_s);
                            write_nv = 1;
                            break;
                        case 2: // lorawan_app_or_join_eui
                            printf("found lorawan_app_or_join_eui\n");
                            strcpy(s_app_eui, (const char*)value_s);
                            write_nv = 1;
                            break;
                        case 3: // lorawan_app_key
                            printf("found lorawan_app_key\n");
                            strcpy(s_app_key, (const char*)value_s);
                            write_nv = 1;
                            break;
                        case 4: // wake_after_sec
                            printf("found wake_after_sec\n");
                            sscanf(value_s, "%hu", &wake_sec);
                            printf("wake_after_sec is %d\n", wake_sec);
                            write_nv = 1;
                            break;
                        case 5: // date
                            printf("found date\n");
                            sscanf(value_s, "%d", &rtc_date);
                            do_date++;
                            break;
                        case 6: // month
                            printf("found month\n");
                            sscanf(value_s, "%d", &rtc_month);
                            do_date++;
                            break;
                        case 7: // year
                            printf("found year\n");
                            sscanf(value_s, "%d", &rtc_year);
                            do_date++;
                            break;
                        case 8: // time
                            printf("found time\n");
                            sscanf(value_s, "%s", rtc_time);
                            do_time++;
                            break;
                        case 9: // random_seed
                            printf("found random_seed\n");
                            sscanf(value_s, "%s", seed_b64);
                            rseed(seed_b64);
                            break;
                        default:
                            printf("unexpected config item\n");
                            break;
                    }
                }

                i++;
                oneitem = itemlist[i];
            } while(oneitem[0]!='-');
            pair_found = 0;
        } else {
            if (not_finished) {
                //printf("ignored row %d\n", row);
            } else {
                printf("finished config file parsing\n");
            }
        }
    } while (not_finished);
    fr = f_close(&fp);
    PRINT_IF_FILE_ERROR("file close error!\n");

    // write date and time to the MSP chip
    if (do_date==3) {
        printf("writing date to RTC chip\n");
        buf[0] = SYSPWR_DATE;
        buf[1] = rtc_year-2000;
        buf[2] = rtc_month;
        buf[3] = rtc_date;
        printf("%d(Y)-%d(M)-%d(D)\n", buf[1], buf[2], buf[3]);
        i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 4, false);
    }
    if (do_time==1) {
        printf("writing time to RTC chip\n");
        buf[0] = SYSPWR_TIME;
        buf[4]=0; // AM
        if ( ((rtc_time[0]=='1') && (rtc_time[1]>'2')) || (rtc_time[0]=='2') ) {
            buf[4]=1; // PM
            if (rtc_time[0]=='1') {
                if (rtc_time[1]>='2') {
                    rtc_time[1]=rtc_time[1]-2;
                    rtc_time[0]='0';
                } else {
                    // shouldn't occur
                }
            } else { // time is 20 hours or greater
                if (rtc_time[1]=='0') {
                    rtc_time[1]='8';
                    rtc_time[0]='0';
                } else if (rtc_time[1]=='1') {
                    rtc_time[1]='9';
                    rtc_time[0]='0';
                } else if (rtc_time[1]=='2') {
                    rtc_time[1]='0';
                    rtc_time[0]='1';
                } else if (rtc_time[1]=='3') {
                    rtc_time[1]='1';
                    rtc_time[0]='1';
                }
            }
        }

        buf[1] = (rtc_time[0]<<4) | (rtc_time[1]&0x0f);
        buf[2] = (rtc_time[3]<<4) | (rtc_time[4]&0x0f);
        buf[3] = (rtc_time[6]<<4) | (rtc_time[7]&0x0f);
        // buf[4] has been set above
        printf("'%s' %02x:%02x:%02x_%d\n", rtc_time, buf[1], buf[2], buf[3], buf[4]);
        i2c_write_blocking(i2c_default, SYSPWR_ADDR, buf, 5, false);
    }

    if (write_nv) {
        nv_erase();

        printf("writing to flash:\n");
        // write random content into the entire buffer first
        for (i=0; i<256; i++) {
            buf[i]=rbyte();
        }
        i=0;
        // format is TLV
        buf[i++] = 0; // region
        buf[i++] = 1;
        buf[i++] = v;

        buf[i++] = 1; // device_eui
        buf[i++] = strlen(sp->device_eui) + 1;
        strncpy(&buf[i], sp->device_eui, buf[i-1]);
        printf("  dev_eui %02x%02x..\n", buf[i], buf[i+1]);
        i=i+buf[i-1];
        buf[i-1]='\0';

        buf[i++] = 2; // app_or_join_eui
        buf[i++] = strlen(sp->app_eui) + 1;
        strncpy(&buf[i], sp->app_eui, buf[i-1]);
        printf("  app_or_join_eui: %02x%02x..\n", buf[i], buf[i+1]);
        i=i+buf[i-1];
        buf[i-1]='\0';

        buf[i++] = 3; // app_key
        buf[i++] = strlen(sp->app_key) + 1;
        strncpy(&buf[i], sp->app_key, buf[i-1]);
        printf("  app_key: %02x%02x..\n", buf[i], buf[i+1]);
        i=i+buf[i-1];
        buf[i-1]='\0';

        buf[i++] = 4; // wake_after_sec
        buf[i++] = 2;
        buf[i++] = (char)((wake_sec >> 8) & 0xff);
        buf[i++] = (char)(wake_sec & 0xff);
        printf("  wake_after_sec: %02x%02x\n", buf[i-2], buf[i-1]);
        syspwr_set_wake_time(wake_sec);

        printf("before scramble: 0x%02x\n", buf[5]);
        rseed(seed); // set the seed
        for (i=0; i<256; i++) {
            rb = rbyte(); 
            buf[i] = buf[i] ^ rb; // scramble the contents
            if (i==5) {
                printf("after scramble with 0x%02x: 0x%02x\n", rb, buf[5]);
            }
        }
        
        nv_write_256(buf);
        
        printf("wrote scrambled config to Flash\n");

        // check
        buf[5] = 0;
        nv_read_256(buf);
        printf("flash read contains: 0x%02x\n", buf[5]);
    }
}

char read_config_nv(void)
{
    int not_finished=1;
    char buf[256];
    int i;
    char ilen;
    int ctot=0;
    char a, b;
    char rb;
    struct lorawan_otaa_settings *sp = &otaa_settings;
    nv_read_256(buf);
    printf("check: flash read contains 0x%02x\n", buf[5]);
    rseed(seed); // set the seed
    for (i=0; i<256; i++) {
        rb = rbyte();
        buf[i] = buf[i] ^ rb; // unscramble the contents
        if (i==5) {
            printf("after unscramble with 0x%02x: 0x%02x\n", rb, buf[5]);
        }
    }

    i=0;
    do {
        if (buf[i]==0) { // region
            if (buf[i+1] == 1) {
                // buf[i+2] contains is region
                switch(buf[i+2]) {
                    case 0:
                        lorawan_region = LORAMAC_REGION_EU868;
                        break;
                    case 1:
                        lorawan_region = LORAMAC_REGION_US915;
                        break;
                    case 2:
                        lorawan_region = LORAMAC_REGION_AU915;
                        break;
                    case 3:
                        lorawan_region = LORAMAC_REGION_IN865;
                        break;
                    default:
                        printf("unknown region!\n");
                        lorawan_region = LORAMAC_REGION_EU868;
                        break;
                }
                ctot++;
                i=i+3;
            } else {
                // error, region length should be 1!
                printf("region length error!\n");
                return(-1);
            }
        } else if (buf[i]==1) { // device_eui
            ilen = buf[i+1];
            if (ilen==17) {
                i=i+2;
                memcpy((void*)sp->device_eui, (const void*)&buf[i], ilen);
                i=i+ilen;
                ctot++;
            } else {
                printf("dev_eui length error!\n");
                return(-1);
            }
        } else if (buf[i]==2) { // app_or_join_eui
            ilen = buf[i+1];
            if (ilen==17) {
                i=i+2;
                memcpy((void*)sp->app_eui, (const void*)&buf[i], ilen);
                i=i+ilen;
                ctot++;
            } else {
                printf("app_or_join_eui length error!\n");
                return(-1);
            }
        } else if (buf[i]==3) { // app_key
            ilen = buf[i+1];
            if (ilen==33) {
                i=i+2;
                memcpy((void*)sp->app_key, (const void*)&buf[i], ilen);
                i=i+ilen;
                ctot++;
            } else {
                printf("app_key length error!\n");
                return(-1);
            }
        } else if (buf[i]==4) { // wake_after_sec
            ilen = buf[i+1];
            if (ilen==2) {
                i=i+2;
                a = buf[i++];
                b = buf[i++];
                wake_sec = (((uint16_t)a)<<8) & 0xff00;
                wake_sec |= ((uint16_t)b) & 0xff;
                ctot++;
            } else {{
                printf("wake_after_sec length error!\n");
            }}
        
        } else if ((i >= 256) || (buf[i] < 0) || (buf[i] > 4))
        {
            not_finished = 0;
            printf("error parsing the flash config!\n");
            return(-1);
        }

        if (ctot>=5) { // we expect ctot to be 5, for 5 pieces of config (id 0 to 4)
            not_finished = 0;
        }

    } while(not_finished);

    return(0);
}

int
sd_test(void)
{
    int retval = 0;
    FRESULT fr;
    unsigned int num;
    char line[100];

    fr = f_mount(&fs, "", 0); // mount the file system
    if (fr==FR_OK) {
        //
    } else {
        printf("sd_test failed! mount error\n");
    }
    // create a new file, deleting the content if it already exists
    fr = f_open(&fp, "test_file_new.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr==FR_OK) {
        f_write(&fp, "SD write test OK!\r\n", 15, &num);
    } else {
        printf("sd_test failed! write error\n");
        retval = -1;
    }

    fr = f_open(&fp, "test.txt", FA_READ);
    if (fr==FR_OK) {
        f_gets(line, sizeof line, &fp);
        // printf("line read: %s\n", line);
        f_close(&fp);
    } else {
        printf("sd_test failed! error opening test.txt\n");
        retval = -1;
    }
    return(retval);
}

// puts the current date/time and the measurement (temperature) as a row into a CSV file.
// The CSV file name contains the year/month/date, so a new file is generated daily.
void
write_record(char* datetime, int v1)
{
    FRESULT fr;
    FILINFO fno;
    unsigned int num;
    char fname[32];
    char rec[128];
    char date[11];
    char file_exists=0;
    strncpy(date, datetime, 10);
    date[10]='\0';

    // build up a file name
    date[4]='_';
    date[7]='_';
    sprintf(fname, "easynode_data_%s.csv", date);

    // build up a record
    date[4]='/';
    date[7]='/';
    sprintf(rec, "%s,%s,%s,%d\r\n", datetime, date, &datetime[11], v1);

    // check if the file exists
    fr = f_stat(fname, &fno);
    if (fr==FR_OK) {
        file_exists = 1;
    }

    // create or open file
    fr = f_open(&fp, fname, FA_OPEN_APPEND | FA_WRITE);
    if (fr==FR_OK) {
        if (file_exists==0) {
            // write header since this is a new file
            f_write(&fp, fields, strlen(fields), &num);
        }
        // write the record
        f_write(&fp, rec, strlen(rec), &num);
        // close the file
        f_close(&fp);
    } else {
        printf("error creating or opening %s!\n", fname);
    }
}

// reads RTC time into a ISO 8601 format ( e.g. 2022-07-23T09:51:00 )
// datetime_string should be a char pointer to at least 20 bytes
void
get_iso_time(char* datetime_string)
{
    syspwr_get_date(datetime_string);
    datetime_string[10]='T';
    syspwr_get_time(&datetime_string[11]);
}

void
poweroff(void) {
#ifdef NVM_USAGE_CHECK
    do {
        sleep_ms(100);
    } while(lorawan_nvm_in_use());
#endif
    KEEP_AWAKE_OFF; // tell the system power controller to remove power from the Pico
}

// ************ main function *******************
int main( void )
{
    FRESULT fr;
    char line[100];
    char datetime_string[24];
    unsigned int num;
    int i;
    int degc=0;
    struct lorawan_otaa_settings *sp = &otaa_settings;
    char buf[256];
    char init_was_pressed = 0;
    int elapsed = 0;

    // first thing: this configures things, especially setting KEEP_AWAKE high
    // so that the system power controller doesn't shut-down the power early
    board_init();

    PICO_LED_ON;
    strcpy((char*)sp->channel_mask, "");

    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    // can now use printf statements

    sleep_ms(5000); // could remove this after debugging, or keep it in

    // print welcome message on the USB UART
    printf("\n\n*** LoRaWAN EasyNode ***\n");
    printf("Built on %s %s\n", __DATE__, __TIME__);

    syspwr_test(); // optional. Tests that there is communication possible with the MSP430 chip

    // powert up LoRa chip, but keep SPI to it disabled
    // so that the SD card can have access to the SPI bus first
    RFM_SPI_DIS;
    POWER_VP_ON;

    therm_init(); // kick off a temperature measurement, we will read it soon.

    // Set LoRaWAN credentials for the LoRaWAN stack
    sp->device_eui = s_device_eui;
    sp->app_eui = s_app_eui;
    sp->app_key = s_app_key;
    sp->channel_mask = LORAWAN_CHANNEL_MASK;

    // check if the user wants configuration from the SD card to be read
    if (INIT_PRESSED) {
        printf("config from SD card has been requested by user\n");
        init_was_pressed = 1;
        lora_stack_nv_erase(); // LoRaWAN stack uses some Flash storage. Erase it in case it's corrupted.
        read_config_file(); // read configuration from SD card, and write to Flash
    } else {
        // SD card config requested to be read, so we use any configuration previously written to Flash
        printf("reading config from Flash\n");
        read_config_nv(); // read from Flash
    }

    // display the config. Should be deleted from a production device of course.
    printf("config:\n");
    printf("  dev_eui: '%s'\n", sp->device_eui);
    printf("  app_or_join_eui: '%s'\n", sp->app_eui);
    printf("  app_key: '%s'\n", sp->app_key);
    printf("  wake_sec: %u\n", wake_sec);

    sd_test(); // optional. Might as well check SD card is functioning while waiting for temperature measurement

    // get the temperature and the time, and write them to the CSV file
    degc = therm_wait_get_value(0); // 0 since we have already initialized the temperature measurement
    get_iso_time(datetime_string);
    write_record(datetime_string, degc);

    // display the data to the USB UART. This could be optionally deleted.
    printf("Written to CSV file:\n");
    printf("  temperature: %d degC\n", degc);
    printf("  date_time: %s\n", datetime_string);

    // LoRaWAN communications:
    printf("Initilizating LoRaWAN ... ");
    // power up the LoRaWAN module
    POWER_VP_ON;
    sleep_ms(100);
    // lorawan_debug(true); // LoRaWAN stack debug

    // initialize the LoRaWAN stack
    if (lorawan_init_otaa(&sx1276_settings, lorawan_region, &otaa_settings) < 0) {
        printf("failed!!!\n");
        PICO_LED_OFF;
        poweroff(); // shutdown since we failed
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    PICO_LED_ON; // lorawan code above sets LED off it seems. bit annoying.
    printf("Joining LoRaWAN network ...");
    lorawan_join(); // the join process writes to Flash.. need to investigate this further

    elapsed = 0; // count how many seconds it takes to join the LoRaWAN network
    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        elapsed++;
        printf(".");
        if (elapsed > 30) {
            printf("\njoin failed\n");
            elapsed = 0;
            PICO_LED_OFF;
            poweroff(); // shutdown since we failed to join
        }
    }
    printf(" joined successfully!\n");

    // loop forever if powered by USB. Otherwise, we will kill our own power by sending a message
    // to the sys_pwr_control device.
    while (1) {
        printf("sending temperature: %d °C (0x%02x)... ", degc, degc);
        if (lorawan_send_unconfirmed(&degc, sizeof(degc), 2) < 0) {
            printf("failed!!!\n");
        } else {
            printf("success!\n");
        }

        // wait for up to 11 seconds for an event
        // (reducing to 5 seconds doesn't work, stack expects some minimum value I have not determined)
        if (lorawan_process_timeout_ms(11000) == 0) {
            // check if a downlink message was received, e.g. request to turn on an output
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1) {
                printf("received %d byte downlink message on port %d: ", receive_length, receive_port);

                for (i = 0; i < receive_length; i++) {
                    printf("%02x", receive_buffer[i]);
                }
                printf("\n");

                // the first byte of the received message controls the two open drain outputs
                // output OUT0_MSP goes to 0V if bit 0 is set to 1
                // output OUT1_MSP goes to 0V if bit 1 is set to 1
                syspwr_set_outputs(receive_buffer[0]);
                gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0]);
            }
        }

        PICO_LED_OFF;
        poweroff(); // shutdown! kills the battery power to the Pico, so the rest of the code won't execute

        sleep_ms(100);
        // none of the code below should execute, because the MSP430 system power controller will
        // have unpowered the Pico microcontroller if running from batteries.
        // However, if the board is being used with a USB connection, then the code will execute.
        RFM_SPI_DIS; // just in case
        sleep_ms(30000); // wait 30 sec
        PICO_LED_ON;
        printf("powered from USB\n");

        // get the temperature and the time, and write them to the CSV file
        degc = therm_wait_get_value(1); // 1 means to first initialize the temperature measurement
        get_iso_time(datetime_string);
        write_record(datetime_string, degc);

        // print the info
        printf("Written to CSV file:\n");
        printf("  temperature: %d degC\n", degc);
        printf("  date_time: %s\n", datetime_string);

    }

    return 0;
}


