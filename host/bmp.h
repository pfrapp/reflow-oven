#ifndef BMP_H
#define BMP_H

#include <stdlib.h>

// Use selected structures and functions from the BMP API.
#define BMP2_OK                                       INT8_C(0)
#define BMP2_E_NULL_PTR                               INT8_C(-1)
/*! @name Temperature range values in integer(32bit, 64bit) and float */
#define BMP2_MIN_TEMP_INT                             INT32_C(-4000)
#define BMP2_MAX_TEMP_INT                             INT32_C(8500)
#define BMP2_MIN_TEMP_DOUBLE                          -40.0f
#define BMP2_MAX_TEMP_DOUBLE                          85.0f
/*! @name Pressure range values in integer and float */
#define BMP2_MIN_PRES_32INT                           UINT32_C(30000)
#define BMP2_MAX_PRES_32INT                           UINT32_C(110000)
#define BMP2_MIN_PRES_64INT                           UINT32_C(30000 * 256)
#define BMP2_MAX_PRES_64INT                           UINT32_C(110000 * 256)
#define BMP2_MIN_PRES_DOUBLE                          30000.0f
#define BMP2_MAX_PRES_DOUBLE                          110000.0f

/*! @name Warning codes */
#define BMP2_W_MIN_TEMP                               INT8_C(1)
#define BMP2_W_MAX_TEMP                               INT8_C(2)
#define BMP2_W_MIN_PRES                               INT8_C(3)
#define BMP2_W_MAX_PRES                               INT8_C(4)


struct bmp2_calib_param
{
    /*! Calibration parameter of temperature data */

    /*! Calibration t1 data */
    uint16_t dig_t1;

    /*! Calibration t2 data */
    int16_t dig_t2;

    /*! Calibration t3 data */
    int16_t dig_t3;

    /*! Calibration parameter of pressure data */

    /*! Calibration p1 data */
    uint16_t dig_p1;

    /*! Calibration p2 data */
    int16_t dig_p2;

    /*! Calibration p3 data */
    int16_t dig_p3;

    /*! Calibration p4 data */
    int16_t dig_p4;

    /*! Calibration p5 data */
    int16_t dig_p5;

    /*! Calibration p6 data */
    int16_t dig_p6;

    /*! Calibration p7 data */
    int16_t dig_p7;

    /*! Calibration p8 data */
    int16_t dig_p8;

    /*! Calibration p9 data */
    int16_t dig_p9;

    /*! Calibration p10 data */
    int8_t dig_p10;

    /*! Fine resolution temperature value */
    int32_t t_fine;
};

struct bmp2_dev
{
    /*! Structure of calibration parameters' */
    struct bmp2_calib_param calib_param;
};

struct bmp2_uncomp_data
{
    /*! Uncompensated temperature data */
    int32_t temperature;

    /*! Uncompensated pressure data */
    uint32_t pressure;
};
struct bmp2_data
{
    /*! Compensated pressure */
    double pressure;

    /*! Compensated temperature */
    double temperature;
};

int8_t bmp2_compensate_data(const struct bmp2_uncomp_data *uncomp_data,
                            struct bmp2_data *comp_data,
                            struct bmp2_dev *dev);

#endif // #ifndef BMP_H
