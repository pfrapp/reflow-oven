#include "bmp.h"


int8_t compensate_pressure(double *comp_pressure,
                                  const struct bmp2_uncomp_data *uncomp_data,
                                  const struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_OK;
    double var1, var2;
    double pressure = 0.0;

    var1 = ((double) dev->calib_param.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) dev->calib_param.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double) dev->calib_param.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double) dev->calib_param.dig_p4) * 65536.0);
    var1 = (((double)dev->calib_param.dig_p3) * var1 * var1 / 524288.0 + ((double)dev->calib_param.dig_p2) * var1) /
           524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) dev->calib_param.dig_p1);

    if (var1 < 0 || var1 > 0)
    {
        pressure = 1048576.0 - (double)uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)dev->calib_param.dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)dev->calib_param.dig_p8) / 32768.0;

        pressure = pressure + (var1 + var2 + ((double)dev->calib_param.dig_p7)) / 16.0;

        if (pressure < BMP2_MIN_PRES_DOUBLE)
        {
            pressure = BMP2_MIN_PRES_DOUBLE;
            rslt = BMP2_W_MIN_PRES;
        }

        if (pressure > BMP2_MAX_PRES_DOUBLE)
        {
            pressure = BMP2_MAX_PRES_DOUBLE;
            rslt = BMP2_W_MAX_PRES;
        }

        (*comp_pressure) = pressure;
    }

    return rslt;
}

int8_t compensate_temperature(double *comp_temperature,
                                     const struct bmp2_uncomp_data *uncomp_data,
                                     struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_OK;
    double var1, var2;
    double temperature;

    var1 = (((double) uncomp_data->temperature) / 16384.0 - ((double) dev->calib_param.dig_t1) / 1024.0) *
           ((double) dev->calib_param.dig_t2);
    var2 =
        ((((double) uncomp_data->temperature) / 131072.0 - ((double) dev->calib_param.dig_t1) / 8192.0) *
         (((double) uncomp_data->temperature) / 131072.0 - ((double) dev->calib_param.dig_t1) / 8192.0)) *
        ((double) dev->calib_param.dig_t3);

    dev->calib_param.t_fine = (int32_t) (var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < BMP2_MIN_TEMP_DOUBLE)
    {
        temperature = BMP2_MIN_TEMP_DOUBLE;
        rslt = BMP2_W_MIN_TEMP;
    }

    if (temperature > BMP2_MAX_TEMP_DOUBLE)
    {
        temperature = BMP2_MAX_TEMP_DOUBLE;
        rslt = BMP2_W_MAX_TEMP;
    }

    (*comp_temperature) = temperature;

    return rslt;
}


int8_t bmp2_compensate_data(const struct bmp2_uncomp_data *uncomp_data,
                            struct bmp2_data *comp_data,
                            struct bmp2_dev *dev)
{
    int8_t rslt;

    if (dev == NULL) {
        return BMP2_E_NULL_PTR;
    }

    if ((rslt == BMP2_OK) && (uncomp_data != NULL) && (comp_data != NULL))
    {
        /* Initialize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;

        rslt = compensate_temperature(&comp_data->temperature, uncomp_data, dev);

        if (rslt == BMP2_OK)
        {
            rslt = compensate_pressure(&comp_data->pressure, uncomp_data, dev);
        }
    }
    else
    {
        rslt = BMP2_E_NULL_PTR;
    }

    return rslt;
}
