#include "logging.h"

int logSignalSample(FILE *log_fid, int index, int time_ms,
                    usb_serial_data_pc_to_tiva *usb_packet_to_tiva,
                    usb_serial_data_tiva_to_pc *usb_packet_from_tiva,
                    int bWriteHeader) {

    if (!log_fid || !usb_packet_to_tiva || !usb_packet_from_tiva) {
        return -1;
    }

    if (bWriteHeader) {
        fprintf(log_fid, "Index,Time (ms),Motor,temp_msb,temp_lsb,temp_xlsb,press_msb,press_lsb,press_xlsb,amp_thermocouple_volt\n");
    }

    //
    fprintf(log_fid, "%05i,%09i,0x%04x,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%08X\n",
            index, time_ms,
            usb_packet_to_tiva->motor,
            usb_packet_from_tiva->temp_msb,
            usb_packet_from_tiva->temp_lsb,
            usb_packet_from_tiva->temp_xlsb,
            usb_packet_from_tiva->press_msb,
            usb_packet_from_tiva->press_lsb,
            usb_packet_from_tiva->press_xlsb,
            usb_packet_from_tiva->amp_thermocouple_voltage);

    return 0;
}
