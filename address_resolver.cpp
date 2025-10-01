/*

uart address resolver v1.1 with debug printfs

adjust power inq delay. make sure comand is right (make sure byte 0 is right, idk.).

Make sure Pin assignments are right!!!! Pins 23-25 are n/a

Add check of return values for power on and off.



Notes:

set counter in nvs loop to go to main loop if no activity.

Add debug printf for power_inq and on/off for response packets.

Changed ack and exec success from 91 to 90. Also modified code to 

Added printfs on line 834 ish to print uart response buffer.


Bugs:

power_inq response check is bad. 
power_cam_on and off are reversed.
Camera side response header addr is 0x90. Make sure code can handle that.


*/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <array>
#include <stdio.h>

#include "NVSOnboard.h"
#include "Settings.h"
#include "pio_tx.pio.h"
#include "pio_rx.pio.h"

extern "C" {
    #include "pico/stdlib.h"
    #include "pico/bootrom.h"
    #include "pico/time.h"
    #include "hardware/flash.h"
    #include "hardware/sync.h"
    #include "hardware/pio.h"
    #include "hardware/uart.h"
    #include "hardware/pwm.h"

}

#define FLASH_WRITE_START (PICO_FLASH_SIZE_BYTES - NVS_SIZE)
#define SETTINGS_MAGIC 0xDEADBEEF
#define NVS_OFFSET (2048 * 1024 - 4096) // Last 4KB of 2MB flash

struct settings_object {
    uint32_t baud_v;
    uint8_t addr1_v;
    uint8_t addr2_v;
    uint8_t ch1_b_id;
    uint8_t ch2_b_id;
    uint8_t resp_head1;
    uint8_t resp_head2;
    uint32_t magic_v;
};

const settings_object default_settings = {
    .baud_v = 9600,
    .addr1_v = 0x81,
    .addr2_v = 0x82,
    .ch1_b_id = 0x80,
    .ch2_b_id = 0x80,
    .resp_head1 = 0x90,
    .resp_head2 = 0x91,
    .magic_v = SETTINGS_MAGIC
};

settings_object new_settings;


// GPIO assignments
constexpr uint tx0 = 0;
constexpr uint rx0 = 1;
constexpr uint tx1 = 4;
constexpr uint rx1 = 5;

constexpr uint PIO_TX_PIN0 = 17;
constexpr uint PIO_RX_PIN0 = 16;
constexpr uint PIO_TX_PIN1 = 19;
constexpr uint PIO_RX_PIN1 = 18;

constexpr uint PIO_DEBUG0 = 10;
constexpr uint PIO_DEBUG1 = 11;

constexpr uint ERROR_PIN = 20;
// constexpr uint RECEIVE_PIN = 15;
// constexpr uint TRANSMIT_PIN = 22;
constexpr uint MASTER_POWER_PIN = 21;

//Switch pins
constexpr uint MASTER_POWER_SW = 12;   
constexpr uint NVS_PIN = 13;   

// Protocol addresses
constexpr size_t BUFF_SIZE = 32;
constexpr uint MAX_PACKET_QUEUE = 9;

//Camera power commands
#define CAM_ACK 0x90, 0x41, 0xff
#define CAM_EXE_SUCCESS 0x90, 0x51, 0xff
constexpr uint8_t CAM_ON_CMD[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xFF};
constexpr uint8_t CAM_OFF_CMD[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xFF};
constexpr uint8_t CAM_INQ_CMD[] = {0x81, 0x09, 0x04, 0x00, 0xFF};
constexpr uint8_t CAM_ON_RP[] = {0x90, 0x50, 0x02, 0xFF};
constexpr uint8_t CAM_OFF_RP[] = {0x90, 0x50, 0x03, 0xFF};

// Buffers
std::array<uint8_t, BUFF_SIZE> main_rx_buff = {};
std::array<uint8_t, BUFF_SIZE> uart0_rx_buff = {};
std::array<uint8_t, BUFF_SIZE> uart1_rx_buff = {};
std::array<uint8_t, BUFF_SIZE> through_rx_buff = {};

std::array<uint8_t, BUFF_SIZE> main_tx_buff = {};
std::array<uint8_t, BUFF_SIZE> uart0_tx_buff = {};
std::array<uint8_t, BUFF_SIZE> uart1_tx_buff = {};
std::array<uint8_t, BUFF_SIZE> through_tx_buff = {};

// Indices and flags
size_t main_rx_index = 0;
size_t uart0_rx_index = 0;
size_t uart1_rx_index = 0;
size_t through_rx_index = 0;

size_t main_tx_index = 0;
size_t uart0_tx_index = 0;
size_t uart1_tx_index = 0;
size_t through_tx_index = 0;

size_t main_flag = 0;
uint8_t ch1_flag = 0;
uint8_t ch2_flag = 0;
uint8_t through_flag = 0;

// PIO reference
PIO pio = pio0;

//timer instance
repeating_timer_t power_pin_timer;

struct Packet {
    uint8_t data[BUFF_SIZE];
    size_t length;
};

// Circular queue for packets to send back over main UART
Packet main_tx_queue[MAX_PACKET_QUEUE];
size_t queue_head = 0;
size_t queue_tail = 0;

void uart_rx_program_init(PIO pio, uint sm, uint offset, uint pin_rx, uint pin_debug, uint baud);
int uart_rx_program_getc(PIO pio, uint sm, uint8_t* out_byte);
bool dequeue_packet(uint8_t* out_data, size_t& out_length);
bool enqueue_packet(const uint8_t* data, size_t length);
void print_array(const std::array<uint8_t, BUFF_SIZE>& arr, size_t len);
void print_c_array(const uint8_t* arr, size_t len);
void get_uart_response(uart_inst_t* interface, std::array<uint8_t, BUFF_SIZE>& buffer, size_t& index, const char name[]);
int cam_power_inq(uart_inst_t *uart);
bool power_cam_on(uart_inst_t *uart);
bool power_cam_off(uart_inst_t *uart);
int gpio_debounce(uint pin, bool normal_state);
int uart_read_with_timeout(uart_inst_t *uart, uint8_t *buf, size_t len, uint32_t timeout_ms);
void clear_newline(char* buffer);
void pwm_start(uint pin, float freq_hz, float duty_percent);
void pwm_stop(uint pin);
void print_new_settings_object();
void set_response_headers();
bool blink_power_pin(repeating_timer_t *rt);



/************************************ Main ******************************************/
int main() {

    stdio_init_all();
    sleep_ms(5000); // Wait for USB serial connection

    printf("Init GPIO...\n");

    // GPIO initialization
    for (uint pin : {ERROR_PIN, MASTER_POWER_PIN}) { //RECEIVE_PIN, TRANSMIT_PIN,
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 1);
    }

    for (uint in_pin : {MASTER_POWER_SW, NVS_PIN}) {
        gpio_init(in_pin);
        gpio_set_dir(in_pin, GPIO_IN);    
        gpio_set_pulls(in_pin, 1, 0);
    }   

    printf("Init flash storage...\n");

    /*
    //Note!!!!!!!!!!!!!!!!!!!!!
    //For first ever write:

    Settings::nvs_init(true);
    Settings::clear();
    Settings::commit();
    */

    //init nvs
    Settings::nvs_init();
    
    //check for vaild settings
    Settings::getMAGIC(&new_settings.magic_v);
    if(new_settings.magic_v == SETTINGS_MAGIC){
        printf("Found valid settings.\n");
        sleep_ms(1000);
        //settings valid
        Settings::getCH1_ADDR(&new_settings.addr1_v);
        Settings::getCH2_ADDR(&new_settings.addr2_v);
        Settings::getCH1_B_ID(&new_settings.ch1_b_id);
        Settings::getCH2_B_ID(&new_settings.ch2_b_id);
        Settings::getBAUD_RATE(&new_settings.baud_v);

    }else{
        printf("No valid settings found...\nWriting defaults\n");
        sleep_ms(1000);
        //write default settings
        Settings::clear();
        Settings::setCH1_ADDR(default_settings.addr1_v);
        Settings::setCH2_ADDR(default_settings.addr2_v);
        Settings::setCH1_B_ID(default_settings.ch1_b_id);
        Settings::setCH2_B_ID(default_settings.ch2_b_id);
        Settings::setBAUD_RATE(default_settings.baud_v);
        Settings::setMAGIC(default_settings.magic_v);
        Settings::commit();
        new_settings = default_settings;
    }
    set_response_headers();
    print_new_settings_object();

    printf("Init serial ports...\n");

    uart_init(uart0, new_settings.baud_v);
    uart_init(uart1, new_settings.baud_v);

    // PIO debug pins
    pio_gpio_init(pio, PIO_DEBUG0);
    pio_gpio_init(pio, PIO_DEBUG1);
    gpio_set_function(PIO_DEBUG0, GPIO_FUNC_PIO0);
    gpio_set_function(PIO_DEBUG1, GPIO_FUNC_PIO0);

    // UART pin function setup
    gpio_set_function(tx0, GPIO_FUNC_UART);
    gpio_set_function(rx0, GPIO_FUNC_UART);
    gpio_set_function(tx1, GPIO_FUNC_UART);
    gpio_set_function(rx1, GPIO_FUNC_UART);

    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(uart0, true);
    uart_set_fifo_enabled(uart1, true);

    // Load PIO programs
    uint offset_rx = pio_add_program(pio, &uart_rx_program);
    uint offset_tx = pio_add_program(pio, &uart_tx_program);

    uart_rx_program_init(pio, 0, offset_rx, PIO_RX_PIN0, PIO_DEBUG0, new_settings.baud_v);
    uart_tx_program_init(pio, 1, offset_tx, PIO_TX_PIN0, new_settings.baud_v);
    uart_rx_program_init(pio, 2, offset_rx, PIO_RX_PIN1, PIO_DEBUG1, new_settings.baud_v);
    uart_tx_program_init(pio, 3, offset_tx, PIO_TX_PIN1, new_settings.baud_v);


    //printf("RS-232 baud rate is %d\n", new_settings.baud_v);

    printf("Scannig for cameras...\n");

    gpio_put(ERROR_PIN, 0);
    //gpio_put(RECEIVE_PIN, 0);
    //gpio_put(TRANSMIT_PIN, 0);
    gpio_put(MASTER_POWER_PIN, 0);

    sleep_ms(1000);

    //inital power check
    int check1 = 0;
    int check2 = 0;
    check1 = cam_power_inq(uart0);
    check2 = cam_power_inq(uart1);

    if((check1 == 1) && (check2 == 1)){
        gpio_put(MASTER_POWER_PIN, 1);

    }else if((check1 == -1) && (check2 == 1)){
        gpio_put(MASTER_POWER_PIN, 1);

    }else if((check1 == 1) && (check2 == -1)){
        gpio_put(MASTER_POWER_PIN, 1);
    }

    printf("Init Success!\n");

    if(gpio_debounce(NVS_PIN, 1)){
        printf("Jumping to Settings mod loop...\n");
        goto set_nvs;
    }

    /************************************ Main Loop ****************************************************/
    main_loop:
    while (true) {

        if (!pio_sm_is_rx_fifo_empty(pio, 0)) {
            //gpio_put(RECEIVE_PIN, 1);

            
            if (main_rx_index >= BUFF_SIZE) {
                printf("Main: rx buffer overflow, resetting index.\n");
                main_rx_index = 0;
            }
            
            uint8_t data = 0;
            int check = uart_rx_program_getc(pio, 0, &data);

            if (check < 0) {
                printf("Framing error from master, data received: 0x%X\n", data);
                main_rx_index = 0;
                main_rx_buff.fill(0);

            } else {
                main_rx_buff[main_rx_index++] = data;
                printf("Received: 0x%X\n", data);

                if (data == 0xFF) {
                    printf("Main: terminator received\nMain: buffer: ");
                    for (size_t i = 0; i < main_rx_index; ++i){
                        printf("0x%02X ", main_rx_buff[i]);
                    }
                    printf("\n");

                    if(main_rx_buff[0] == new_settings.addr1_v){
                        printf("Printing on if1\n");
                        main_rx_buff[0] = 0x81;
                        std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, uart0_tx_buff.begin());
                        ch1_flag = main_rx_index;
                        
                    }else if(main_rx_buff[0] == new_settings.addr2_v){
                        printf("Printing on if2\n");
                        main_rx_buff[0] = 0x81;
                        std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, uart1_tx_buff.begin());
                        ch2_flag = main_rx_index;
                        
                    }else if(main_rx_buff[0] == (0x80 || 0x88)){
                        printf("Broadcasting\n");
                        std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, uart0_tx_buff.begin());
                        uart0_tx_buff[0] = new_settings.ch1_b_id;
                        std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, uart1_tx_buff.begin());
                        uart1_tx_buff[0] = new_settings.ch1_b_id;
                        std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, through_tx_buff.begin());
                        ch1_flag = ch2_flag = through_flag = main_rx_index;

                    }else{
                        if ((main_rx_buff[0] & 0xF0) == 0x80) {
                            printf("Writing to if3\n");
                            std::copy(main_rx_buff.begin(), main_rx_buff.begin() + main_rx_index, through_tx_buff.begin());
                            through_flag = main_rx_index;
                        } else {
                            gpio_put(ERROR_PIN, 1);
                            printf("Invalid command from master, addr: 0x%x\n", main_rx_buff[0]);
                            gpio_put(ERROR_PIN, 0);
                        }
                    }

                    main_rx_index = 0;
                    main_rx_buff.fill(0);
                }
            }

            //gpio_put(RECEIVE_PIN, 0);

        }

        // Transmit handling
        if (ch1_flag) {
            while (uart_is_writable(uart0) && uart0_tx_index < ch1_flag){
                uart_putc(uart0, uart0_tx_buff[uart0_tx_index]);
                printf("if1: sent 0x%X\n", uart0_tx_buff[uart0_tx_index]);
                uart0_tx_index++;

            }

            if (uart0_tx_index >= ch1_flag){
                uart0_tx_index = ch1_flag = 0;
            }

        }

        if (ch2_flag) {
            while (uart_is_writable(uart1) && uart1_tx_index < ch2_flag){
                uart_putc(uart1, uart1_tx_buff[uart1_tx_index]);
                printf("if2: sent 0x%X\n", uart1_tx_buff[uart1_tx_index]);
                uart1_tx_index++;
            }

            if (uart1_tx_index >= ch2_flag){
                uart1_tx_index = ch2_flag = 0;
            }
        }

        if (through_flag) {
            while (through_tx_index < through_flag && !pio_sm_is_tx_fifo_full(pio, 3)){
                pio_sm_put(pio, 3, through_tx_buff[through_tx_index]);
                printf("if3: sent 0x%X\n", through_tx_buff[through_tx_index]);
                through_tx_index++;

            }

            if (through_tx_index >= through_flag){
                through_tx_index = through_flag = 0;
            }
        }

        // Receivee handling 
        get_uart_response(uart0, uart0_rx_buff, uart0_rx_index, "if1");
        get_uart_response(uart1, uart1_rx_buff, uart1_rx_index, "if2");
        
        while (!pio_sm_is_rx_fifo_empty(pio, 2)) { 

            if (through_rx_index >= BUFF_SIZE) {
                printf("if3: buffer overflow, resetting index.\n");
                through_rx_buff.fill(0);
                through_rx_index = 0;
            }

            uint8_t data = 0;
            int check = uart_rx_program_getc(pio, 2, &data);

            if (check < 0) {
                printf("if3: Framing error. Data: 0x%X\n", data);
                through_rx_buff.fill(0);
                through_rx_index = 0;

            } else {
                through_rx_buff[through_rx_index] = data;
                printf("if3: received 0x%X\n", through_rx_buff[through_rx_index]);
                if(through_rx_buff[through_rx_index] == 0xFF){
                    if((through_rx_buff[0] & 0xF0) == 0x90){
                        printf("if3: terminator received, attempting to enqueue packet: ");
                        print_array(through_rx_buff, through_rx_index + 1);
                        printf("\n");
                        if(enqueue_packet(through_rx_buff.data(), through_rx_index +1)){
                            printf("if3: enqueue success!\n");
                        }
                        else {
                            printf("if3: enqueue failed\n");
                        }

                    }else{
                        printf("if3: received bad response addr 0x%02X\n", through_rx_buff[0]);
                    }

                    through_rx_index = 0;
                    through_rx_buff.fill(0);

                }else{
                through_rx_index++;
                }
            }

        }

        //Dequeue package if not already
        if (!main_flag){   
            if (dequeue_packet(main_tx_buff.data(), main_flag)){
                printf("Dequeued: ");
                print_array(main_tx_buff, main_flag);
                printf("\n");
                main_tx_index = 0;
            }

        }

        //Transmit package
        if (main_flag){ 
            while (main_tx_index < (main_flag) && !pio_sm_is_tx_fifo_full(pio, 1) && main_flag){
                pio_sm_put(pio, 1, main_tx_buff[main_tx_index++]);
            }

            if(main_tx_index >= main_flag){
                main_tx_index = main_flag = 0;
                through_rx_buff.fill(0);
            }
            
        }

        //master power options
        if(gpio_debounce(MASTER_POWER_SW, 1)){
            printf("Changing camera power state\n");
            bool cam_is_on = gpio_get(MASTER_POWER_PIN);
            add_repeating_timer_ms(500, blink_power_pin, NULL, &power_pin_timer);

            if(!cam_is_on){
                printf("Powering both cameras off!\n");
                power_cam_off(uart0);
                power_cam_off(uart1);
                gpio_put(MASTER_POWER_PIN, 0);
                while(gpio_debounce(MASTER_POWER_SW, 1)){
                    sleep_ms(100);
                }

            }else{
                printf("Powering both cameras on!\n");
                power_cam_on(uart0);
                power_cam_on(uart1);
                gpio_put(MASTER_POWER_PIN, 1);
                while(gpio_debounce(MASTER_POWER_SW, 1)){
                    sleep_ms(100);
                }

            }
            cancel_repeating_timer(&power_pin_timer);
            gpio_put(MASTER_POWER_PIN, !cam_is_on);
            printf("done changing camera power.\n");

        }
    

        if(gpio_debounce(NVS_PIN, 1)){
            // sleep_ms(2000);
            // if(gpio_debounce(NVS_PIN, 1)){
                goto set_nvs;
            // }
        }
    }



    /*********************************** Set NVS loop ***************************************************/
    set_nvs:
    while(1){

        bool save_is_cam_on_in_nvs = gpio_get(MASTER_POWER_PIN);
        add_repeating_timer_ms(250, blink_power_pin, NULL, &power_pin_timer);


        // Wait for USB terminal connection
        int nvs_timout_value = 30;
        int nvs_timout_counter = 0;
        while (!stdio_usb_connected() && nvs_timout_counter < nvs_timout_value) {
            sleep_ms(100);
            nvs_timout_counter++;
        }

        if(nvs_timout_counter >= nvs_timout_value){
            cancel_repeating_timer(&power_pin_timer);
            gpio_put(MASTER_POWER_PIN, save_is_cam_on_in_nvs);
            goto main_loop;

        }

        printf("\nYou have entered config mode.\nIf this was unintentional: wait a few seconds, then unplug\n");      
        sleep_ms(1000);

        //init nvs
        Settings::nvs_init();
        
        //check for vaild settings
        Settings::getMAGIC(&new_settings.magic_v);
        if(new_settings.magic_v == SETTINGS_MAGIC){
            printf("Found valid settings.\n");
            sleep_ms(1000);
            //settings valid
            Settings::getCH1_ADDR(&new_settings.addr1_v);
            Settings::getCH2_ADDR(&new_settings.addr2_v);
            Settings::getCH1_B_ID(&new_settings.ch1_b_id);
            Settings::getCH2_B_ID(&new_settings.ch2_b_id);
            Settings::getBAUD_RATE(&new_settings.baud_v);
        
        }else{
            printf("No valid settings found...\nWriting defaults\n");
            sleep_ms(1000);
            //write default settings
            Settings::setCH1_ADDR(default_settings.addr1_v);
            Settings::setCH2_ADDR(default_settings.addr2_v);
            Settings::setCH1_B_ID(default_settings.ch1_b_id);
            Settings::setCH2_B_ID(default_settings.ch2_b_id);
            Settings::setBAUD_RATE(default_settings.baud_v);
            Settings::setMAGIC(default_settings.magic_v);
            Settings::commit();

            new_settings = default_settings;
            
        }
        set_response_headers();
        print_new_settings_object();
                
        char buf[64];
        
        while (true) {
            printf("Commands: 'list', 'edit', 'default', 'update', 'exit'\n>>> ");
            fgets(buf, sizeof(buf), stdin);
            clear_newline(buf);

            if (strcmp(buf, "edit") == 0) {
                settings_object temp = new_settings;
                printf("edit\n");

                printf("    Send 'x' to skip changing an entry\n\n");

                printf("    Enter new CH1_ID (1-7): ");
                fgets(buf, sizeof(buf), stdin);
                if(buf[0] != 'x'){
                    temp.addr1_v = atoi(buf);
                    temp.addr1_v = (temp.addr1_v & 0x07) | 0x80;
                    if(temp.addr1_v == 0x80) temp.addr1_v = 0x81;
                }
                printf("%d\n", (temp.addr1_v & 0x0f));


                printf("    Enter new CH2_ID (1-7): ");
                fgets(buf, sizeof(buf), stdin);
                if(buf[0] != 'x'){
                    temp.addr2_v = atoi(buf);
                    temp.addr2_v = (temp.addr2_v & 0x07) | 0x80;
                    if(temp.addr2_v == 0x80) temp.addr2_v = 0x82;
                }
                printf("%d\n", (temp.addr2_v & 0x0f));

                printf("    Enter new CH1 broadcast ID (0 or 8): ");
                fgets(buf, sizeof(buf), stdin);
                if(buf[0] != 'x'){
                    temp.ch1_b_id = atoi(buf);
                    temp.ch1_b_id = (temp.ch1_b_id & 0x08) | 0x80;
                }
                printf("%d\n", (temp.ch1_b_id & 0x0f));
                
                printf("    Enter new CH2 broadcast ID (0 or 8): ");
                fgets(buf, sizeof(buf), stdin);
                if(buf[0] != 'x'){
                    temp.ch2_b_id = atoi(buf);
                    temp.ch2_b_id = (temp.ch2_b_id & 0x08) | 0x80;
                }
                printf("%d\n", (temp.ch2_b_id & 0x0f));

                printf("    Caution: Baud rate is not fully sanitized! \n");
                printf("    You have the power to enter non-standard baud rates >=300, but there are no garuntees they will work!\n");
                printf("    Enter new Baud rate: ");
                fgets(buf, sizeof(buf), stdin);
                if(buf[0] != 'x'){
                    temp.baud_v = atoi(buf);
                }
                if(temp.baud_v < 300) temp.baud_v = 300;
                printf("%d\n", temp.baud_v);

                temp.magic_v = SETTINGS_MAGIC;

                printf("    Save these settings? (y/n): ");
                fgets(buf, sizeof(buf), stdin);
                clear_newline(buf);

                if (strcmp(buf, "y") == 0 || strcmp(buf, "Y") == 0) {

                    Settings::setCH1_ADDR(temp.addr1_v);
                    Settings::setCH2_ADDR(temp.addr2_v);
                    Settings::setCH1_B_ID(temp.ch1_b_id);
                    Settings::setCH2_B_ID(temp.ch2_b_id);
                    Settings::setBAUD_RATE(temp.baud_v);
                    Settings::setMAGIC(temp.magic_v);
                    sleep_ms(1000);

                    if(Settings::commit() == NVS_OK){
                        printf("y\n        Changes saved!\n");
                    }
                    else{
                        printf("        Error: write to flash failed.\n    Changes will not survive reboot!");
                    }
                    new_settings = temp;
                    set_response_headers();
                    print_new_settings_object();
                
                } else {
                    printf("Changes discarded.\n");
                }

            }else if(strcmp(buf, "exit") == 0){
                printf("Retured to normal operation\n\n\n");
                cancel_repeating_timer(&power_pin_timer);
                sleep_ms(100);
                gpio_put(MASTER_POWER_PIN, save_is_cam_on_in_nvs);
                goto main_loop;

            }else if (strcmp(buf, "update") == 0) {
                printf("Rebooting into bootloader...\n");
                sleep_ms(500);
                reset_usb_boot(0, 0);
            }else if (strcmp(buf, "default") == 0){
                printf("Write default Settings? (y/n) ");
                fgets(buf, sizeof(buf), stdin);
                clear_newline(buf);
                if (strcmp(buf, "y") == 0 || strcmp(buf, "Y") == 0) {
                    printf("y\n");
                    sleep_ms(1000);
                    //write default settings
                    Settings::setCH1_ADDR(default_settings.addr1_v);
                    Settings::setCH2_ADDR(default_settings.addr2_v);
                    Settings::setCH1_B_ID(default_settings.ch1_b_id);
                    Settings::setCH2_B_ID(default_settings.ch2_b_id);
                    Settings::setBAUD_RATE(default_settings.baud_v);
                    Settings::setMAGIC(default_settings.magic_v);
                    Settings::commit();
                    set_response_headers();
                    new_settings = default_settings;
                    printf("Success!\n");
                    print_new_settings_object();

                }else{
                    printf("\n\nAborting operation\n\n");
                }
            
            }else if(strcmp(buf, "list") == 0){
                print_new_settings_object();

            }else{
                printf("Unknown command.\nSend COMMAND<lf><cr>\n");
            }

        }

    }

    return 0;

}


/***************************************Special Functions*************************************************/

/************************************** PIO RX Program Helper ********************************************/
int uart_rx_program_getc(PIO pio, uint sm, uint8_t* out_byte) {
    uint32_t raw = pio_sm_get(pio, sm);
    raw >>= 23;
    uint8_t data = raw & 0xFF;
    bool stop_bit = (raw >> 8) & 1;

    //printf("raw: 0x%03x stop bit: %d data: 0x%02x\n", raw, stop_bit, data);

    if (!stop_bit) return -1; // Framing error

    *out_byte = data;
    return 0;
}

bool enqueue_packet(const uint8_t* data, size_t length) {
    if (length == 0 || length > BUFF_SIZE) return false;
    size_t next_tail = (queue_tail + 1) % MAX_PACKET_QUEUE;
    if (next_tail == queue_head) return false;

    memcpy(main_tx_queue[queue_tail].data, data, length);
    main_tx_queue[queue_tail].length = length;
    queue_tail = next_tail;
    return true;
}

bool dequeue_packet(uint8_t* out_data, size_t& out_length) {
    if (queue_head == queue_tail) return false; // Queue empty
    memcpy(out_data, main_tx_queue[queue_head].data, main_tx_queue[queue_head].length);
    out_length = main_tx_queue[queue_head].length;
    queue_head = (queue_head + 1) % MAX_PACKET_QUEUE;
    return true;
}

void print_array(const std::array<uint8_t, BUFF_SIZE>& arr, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", arr[i]);  // Print as 2-digit hex with leading 0
    }
}

void print_c_array(const uint8_t* arr, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", arr[i]);  // Print as 2-digit hex with leading 0
    }
}

void get_uart_response(uart_inst_t* interface, std::array<uint8_t, BUFF_SIZE>& buffer, size_t& index, const char name[]){

    uint8_t data = 0;
    while (uart_is_readable(interface)){ //get any available data
        data = uart_getc(interface);
        printf("%s: received 0x%X\n", name, data);

        if (index >= BUFF_SIZE) {  //avoid overflow
            printf("%s: buffer overflow, resetting index.\n", name);
            buffer.fill(0);
            index = 0;
            return;
        }

        buffer[index] = data;
        
        if((buffer[index] == 0xFF)){
            printf("%s: terminator received, attempting to enqueue packet: ", name);
            print_array(buffer, index + 1);
            printf("\n");
            if(buffer[0] == 0x90){
                
                if(interface == uart0){
                    buffer[0] = new_settings.resp_head1;
                }else{
                    buffer[0] = new_settings.resp_head2;
                }

                printf("enqueueing ");
                print_array(buffer, index + 1);
                printf("\n");

                if(enqueue_packet(buffer.data(), index + 1)){
                    printf("%s: enqueue success!\n", name);
                    buffer.fill(0);
                    index = 0;
                    return;
                }
                else {
                    printf("%s: enqueue failed\n", name);
                    buffer.fill(0);
                    index = 0;
                    return;
                }
            }else{
                printf("%s: received bad response (should start with 0x90): 0x%02X\nBuffer = ", name, buffer[0]);
                print_array(buffer, index + 1);
                printf("\n");
                buffer.fill(0);
                index = 0;
                return;
            }            

        }else{
            index++;
        }

    }
    return;

}


//Camera power functions
int cam_power_inq(uart_inst_t *uart){
    int count = 0;
    int index = 0;
    int num = -1;
    bool terminator = 0;
    uint8_t response[6];
    memset(response, 0, sizeof(response));
    if(uart == uart0){
        num = 1;
    }else{
        num = 2;
    }

    uart_write_blocking(uart, CAM_INQ_CMD, sizeof(CAM_INQ_CMD));
    while(count < 5000 && !terminator){
        if(uart_is_readable(uart)){
            response[index] = uart_getc(uart);

            if(index > 5){
                printf("if%d: Power inq failed. Buffer overflow.\n", num);
                index = 0;
                count = 5001;
                return -1;
            }
            if(response[index] == 0xFF){
                terminator = 1;
                printf("if%d: Power inq: Terminator recieved.\n", num);
            }
            
            index++;

        }

        count++;

        if(count >= 5000){
            printf("if%d: Power inq failed: Timeout.\n", num);
            return -1;
        }

        sleep_ms(1);

    }

    if(terminator){

        printf("recieved: ");
        print_c_array(response, sizeof(response));
        printf("\n");
        
        if(memcmp(response, CAM_ON_RP, sizeof(CAM_ON_RP)) == 0){
            printf("if%d: Camera is on\n", num);
            return 1;
        }else if(memcmp(response, CAM_OFF_RP, sizeof(CAM_OFF_RP)) == 0){
            printf("if%d: Camera is off\n", num);
            return 0;
        }else{
            printf("if%d: Power inq response unknown. Assuming off.\n", num);
            return 0;
        }
        
    }
    return -1;
}

bool power_cam_on(uart_inst_t *uart){
    uint8_t buf[6];
    uint8_t expected_response[6]={CAM_ACK, CAM_EXE_SUCCESS};
    uart_write_blocking(uart, CAM_ON_CMD, sizeof(CAM_ON_CMD));
    if (uart_read_with_timeout(uart, buf, 6, 10000) == 1 && memcmp(buf, expected_response, 6) == 0) {
        printf("Powered on, recieved: ");
        print_c_array(buf, sizeof(buf));
        printf("\n");
        return true;
    }
    return false;
}

bool power_cam_off(uart_inst_t *uart){
    uint8_t buf[6];
    uint8_t expected_response[6]={CAM_ACK, CAM_EXE_SUCCESS};
    uart_write_blocking(uart, CAM_OFF_CMD, sizeof(CAM_OFF_CMD));
    if (uart_read_with_timeout(uart, buf, 6, 10000) == 1 && memcmp(buf, expected_response, 6) == 0) {
        printf("Powered off, recieved: ");
        print_c_array(buf, sizeof(buf));
        printf("\n");
        return true;
    }
    return false;
    
}


//return 1 if pin != normal_state else return 0
int gpio_debounce(uint pin, bool normal_state){
    if(!gpio_get(pin) == normal_state){
        sleep_ms(20);
        if(!gpio_get(pin) == normal_state){
            return 1;
        }
    }

    return 0;

}

//return 0 if timout, 1 if success
int uart_read_with_timeout(uart_inst_t *uart, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    size_t received = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (received < len) {
        if (uart_is_readable(uart)) {
            buf[received++] = uart_getc(uart);
        }
        // Bail out if we’ve hit the timeout
        if (time_reached(deadline)) return 0; // timeout
        sleep_ms(1); // brief yield
    }
    return 1; // success
}

void clear_newline(char* buffer) {
    buffer[strcspn(buffer, "\r\n")] = '\0';
};

// Start PWM on a given pin with frequency (Hz) and duty cycle (%)
void pwm_start(uint pin, float freq_hz, float duty_percent) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);

    uint32_t clock = clock_get_hz(clk_sys); // 125 MHz default

    // Pick a TOP value that gives good resolution (max 16-bit)
    uint32_t top = 65535;

    // Calculate divider that will give the requested frequency
    float divider = (float)clock / (freq_hz * (top + 1));

    // If divider is too small (<1.0), shrink TOP instead
    if (divider < 1.0f) {
        divider = 1.0f;
        top = (uint32_t)((float)clock / (freq_hz * divider) - 1);
        if (top > 65535) top = 65535;
    }

    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, top);

    // Duty cycle
    uint32_t level = (uint32_t)((top + 1) * duty_percent / 100.0f);
    pwm_set_chan_level(slice_num, channel, level);

    pwm_set_enabled(slice_num, true);
}

// Stop PWM and force pin low
void pwm_stop(uint pin) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);

    pwm_set_enabled(slice_num, false);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0); // turn LED off
}

void print_new_settings_object(){

    printf("\nCurrent settings (user):\n    CH1_ID: %d\n    CH2_ID: %d\n    Baud: %d\n", 
        (new_settings.addr1_v & 0x0f), (new_settings.addr2_v & 0x0f), new_settings.baud_v);
    printf("    CH1 broadcast ID: %X\n    CH2 broadcast ID: %X\n", (new_settings.ch1_b_id & 0x0f), (new_settings.ch2_b_id & 0x0f));

    printf("Current settings (real):\n    CH1_ID: %X\n    CH2_ID: %X\n    Baud: %d\n", 
        new_settings.addr1_v, new_settings.addr2_v, new_settings.baud_v);
    printf("    CH1 broadcast ID: %X\n    CH2 broadcast ID: %X\n\n", new_settings.ch1_b_id, new_settings.ch2_b_id);
}

void set_response_headers(){
    new_settings.resp_head1 = (new_settings.addr1_v | 0x10) - 1;
    new_settings.resp_head2 = (new_settings.addr2_v | 0x10) - 1;
    printf("Response IDs: %X, %X", new_settings.resp_head1, new_settings.resp_head2);

}

// Callback runs in interrupt context
bool blink_power_pin(repeating_timer_t *rt) {
    static bool state = false;
    state = !state;
    gpio_put(MASTER_POWER_PIN, state);
    return 1; // return true to keep repeating
}
