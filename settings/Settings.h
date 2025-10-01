#pragma once
#include "../nvs_lib/NVSOnboard.h"
#include "Settings_keys.h"

class Settings {
public:
    static void nvs_init(bool reset = false) {
        nvs = NVSOnboard::getInstance(reset);
    }

    static void setCH1_ADDR(uint8_t addr) {
        nvs->set_u8(CH1_ADDR, addr);
    }

    static void getCH1_ADDR(uint8_t* out) {
        nvs->get_u8(CH1_ADDR, out);
    }

    static void setCH2_ADDR(uint8_t addr) {
        nvs->set_u8(CH2_ADDR, addr);
    }

    static void getCH2_ADDR(uint8_t* out) {
        nvs->get_u8(CH2_ADDR, out);
    }
    


    static void setCH1_B_ID(uint8_t addr) {
        nvs->set_u8(CH1_B_ID, addr);
    }

    static void getCH1_B_ID(uint8_t* out) {
        nvs->get_u8(CH1_B_ID, out);
    }

    static void setCH2_B_ID(uint8_t addr) {
        nvs->set_u8(CH2_B_ID, addr);
    }

    static void getCH2_B_ID(uint8_t* out) {
        nvs->get_u8(CH2_B_ID, out);
    }



    static void setBAUD_RATE(uint32_t baud) {
        nvs->set_u32(BAUD_RATE, baud);
    }

    static void getBAUD_RATE(uint32_t* out) {
        nvs->get_u32(BAUD_RATE, out);
    }
    
    static void setMAGIC(uint32_t magic) {
        nvs->set_u32(MAGIC, magic);
    }

    static void getMAGIC(uint32_t* out) {
        nvs->get_u32(MAGIC, out);
    }

    static nvs_err_t commit() {
        return nvs->commit();
    }

    static void clear() {
        nvs->clear();
        nvs->commit();
    }

private:
    static NVSOnboard* nvs;
};
