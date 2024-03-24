#include "logging.h"

int logSignalSample(FILE *log_fid, int index, int time_ms,
                    usb_serial_data_pc_to_tiva *usb_packet_to_tiva,
                    usb_serial_data_tiva_to_pc *usb_packet_from_tiva,
                    int bWriteHeader) {

    if (!log_fid || !usb_packet_to_tiva || !usb_packet_from_tiva) {
        return -1;
    }

    if (bWriteHeader) {
        fprintf(log_fid, "Index,Time (ms),Motor,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,QuadraturePosition,QuadratureVelocity\n");
    }

    //
    fprintf(log_fid, "%05i,%09i,0x%04x,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%08X,0x%08X\n",
            index, time_ms,
            usb_packet_to_tiva->motor,
            usb_packet_from_tiva->accel_x,
            usb_packet_from_tiva->accel_y,
            usb_packet_from_tiva->accel_z,
            usb_packet_from_tiva->gyro_x,
            usb_packet_from_tiva->gyro_y,
            usb_packet_from_tiva->gyro_z,
            usb_packet_from_tiva->quadrature_position,
            usb_packet_from_tiva->quadrature_velocity);

    return 0;
}
