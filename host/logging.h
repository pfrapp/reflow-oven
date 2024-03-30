#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h> // For FILE
#include "usb_serial_data.h"


int logSignalSample(FILE *log_fid, int index, int time_ms,
                    usb_serial_data_pc_to_tiva *usb_packet_to_tiva,
                    usb_serial_data_tiva_to_pc *usb_packet_from_tiva,
                    int bWriteHeader);

#endif // #ifndef LOGGING_H
