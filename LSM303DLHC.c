// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// LSM303DLHC
// This code is designed to work with the LSM303DLHC_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/products

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

const char* bus = "/dev/i2c-1";

struct accl_data {
    int64_t xAccl;
    int64_t yAccl;
    int64_t zAccl;
};

struct magn_data {
    int64_t xMag;
    int64_t yMag;
    int64_t zMag;
};

int8_t open_dev(int* handle) {
	// Create I2C bus
	if((*handle = open(bus, O_RDWR)) < 0) {
		printf("Failed to open the bus. \n");
        return 1;
	}
    return 0;
}

int8_t get_accl_data(int file, struct accl_data* out) {
    if (out == NULL) {
        fprintf(stderr, "Cannot get accelerometer data: output is null\n");
        return 1;
    }

    // Get I2C device, LSM303DLHC ACCELERO I2C address is 0x19(25)
    ioctl(file, I2C_SLAVE, 0x19);

    // Select control register1(0x20)
    /** 
    -------------------------------------
    | d1 | d2 | d3 | d4 | l1 | z | y | x|
    -------------------------------------

    Where bits
        * d1, d2, d3, d4 set the data transfer rate to 10 Hz
        * l1 is 0 to use normal power consumption
        * z, y, x are 1 to enable all axis for the accelerometer

    0010 0111 == 0x27
    */

    char config[2] = {0};
    config[0] = 0x20;
    config[1] = 0x27;
    write(file, config, 2);

    // Select control register4(0x23)
    // Full scale +/- 2g, continuous update(0x00)

    /**

    --------------------------------------------
    | bdu | ble | fs1 | fs0 | hr | 0 | 0 | SIM |
    --------------------------------------------

    Where bits
        * bdu sets block data update state to either 0 (continuous)
        or 1 which only updates when MSB or LSB
        * ble specifies either big endian or little endian data selection
        0 -> LSB is at lower address, 1 -> MSB is at lower address
        * fs1, fs0: "Full-scale selection", what is this?
        00: +/- 2G, 01: +/- 4G, 10: +/- 8G, 11: +/- 16G 
        * HR high resolution output mode. 0: disable, 1: enable
        * SIM: "SPI serial interface mode selection", what is this?
        0: 4-wire access, 1: 3-wire access

        Set all to 0
    */

    config[0] = 0x23;
    config[1] = 0x00;
    write(file, config, 2);
    usleep(1000 * 250);

    // Read 6 bytes of data
    // lsb first
    // Read xAccl lsb data from register(0x28)
    char reg[1] = {0x28};
    write(file, reg, 1);
    char data[1] = {0};
    if(read(file, data, 1) != 1)
    {
        printf("Error : Input/output Error in reading accelerometer data\n");
        return 1;
    }

    char data_0 = data[0];

    // Read xAccl msb data from register(0x29)
    reg[0] = 0x29;
    write(file, reg, 1);
    read(file, data, 1);
    char data_1 = data[0];

    // Read yAccl lsb data from register(0x2A)
    reg[0] = 0x2A;
    write(file, reg, 1);
    read(file, data, 1);
    char data_2 = data[0];

    // Read yAccl msb data from register(0x2B)
    reg[0] = 0x2B;
    write(file, reg, 1);
    read(file, data, 1);
    char data_3 = data[0];

    // Read zAccl lsb data from register(0x2C)
    reg[0] = 0x2C;
    write(file, reg, 1);
    read(file, data, 1);
    char data_4 = data[0];

    // Read zAccl msb data from register(0x2D)
    reg[0] = 0x2D;
    write(file, reg, 1);
    read(file, data, 1);
    char data_5 = data[0];

    // Convert the data
    int xAccl = (data_1 * 256 + data_0);
    if(xAccl > 32767)
    {
        xAccl -= 65536;
    }

    int yAccl = (data_3 * 256 + data_2);
    if(yAccl > 32767)
    {
        yAccl -= 65536;
    }

    int zAccl = (data_5 * 256 + data_4);
    if(zAccl > 32767)
    {
        zAccl -= 65536;
    }

    out->xAccl = xAccl;
    out->yAccl = yAccl;
    out->zAccl = zAccl;

    return 0;
}

int8_t get_magn_data(int file, struct magn_data* out) {
    if (out == NULL) {
        fprintf(stderr, "Cannot get magnetometer data: output is null\n");
        return 1;
    }

    char config[2] = {0};
    char reg[1] = {0x28};
    char data[1] = {0};

    // Get I2C device, LSM303DLHC MAGNETO I2C address is 0x1E(30)
    ioctl(file, I2C_SLAVE, 0x1E);

    // Select MR register(0x02)
    /**

    -------------------------------------
    | 0 | 0 | 0 | 0 | 0 | 0 | md1 | md0 |
    -------------------------------------

    where bits
        * md1, md0: magnetic sensor operating mode
        00: continous-conversion mode
        01: single-conversion mode
        10, 11: Sleep mode. Device is placed in sleep mode
    */

    config[0] = 0x02;
    config[1] = 0x00;
    write(file, config, 2);

    // Select CRA register(0x00)
    /**
    ---------------------------------------------
    | temp_en | 0 | 0 | DO2 | DO1 | DO0 | 0 | 0 |
    ---------------------------------------------

    where bits
        * temp_en: temperature sensor enable. 0: disabled, 1: enabled
        * DO2, DO1, DO0: Data output rate bits
        100: 15Hz
    
    0001 0000 == 0x10
    */

    config[0] = 0x00;
    config[1] = 0x10;
    write(file, config, 2);

    // Select CRB register(0x01)
    // Set gain = +/- 1.3g(0x20)

    /**
    ---------------------------------------
    | GN2 | GN1 | GN0 | 0 | 0 | 0 | 0 | 0 |
    ---------------------------------------

    where bits
        GN2, GN1, GN0: gain configuration bits
        001: sensor input field range [Gauss]: +/- 1.3,
             gain X, Y, Z [LSB/Gauss]: 1100
             gain Z [LSB/Gauss]: 980
             output range: 0xF800 - 0x07FF (-2048 - 2047)
        0010 0000 == 0x20
    */

    config[0] = 0x01;
    config[1] = 0x20;
    write(file, config, 2);
    usleep(1000 * 250);

    // Read 6 bytes of data
    // msb first
    // Read xMag msb data from register(0x03)
    reg[0] = 0x03;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_0 = data[0];

    // Read xMag lsb data from register(0x04)
    reg[0] = 0x04;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_1 = data[0];

    // Read yMag msb data from register(0x05)
    reg[0] = 0x05;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_2 = data[0];

    // Read yMag lsb data from register(0x06)
    reg[0] = 0x06;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_3 = data[0];

    // Read zMag msb data from register(0x07)
    reg[0] = 0x07;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_4 = data[0];

    // Read zMag lsb data from register(0x08)
    reg[0] = 0x08;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_5 = data[0];

    // Convert the data
    int xMag = (data1_0 * 256 + data1_1);
    if(xMag > 32767)
    {
        xMag -= 65536;
    }	

    int yMag = (data1_4 * 256 + data1_5) ;
    if(yMag > 32767)
    {
        yMag -= 65536;
    }

    int zMag = (data1_2 * 256 + data1_3) ;
    if(zMag > 32767)
    {
        zMag -= 65536;
    }

    out->xMag = xMag;
    out->yMag = yMag;
    out->zMag = zMag;

    return 0;
}

int main() {
    // Create I2C bus
	int file;
    int8_t ret = open_dev(&file);
    if (ret != 0) {
        fprintf(stderr, "Exiting due to failure in opening I2C bus\n");
        return 1;
    }

    struct accl_data accl;
    struct magn_data magn;

    struct accl_data accl_sum;
    struct magn_data magn_sum;

    double a_sum_x = 0;
    double a_sum_y = 0;
    double a_sum_z = 0;

    double m_sum_x = 0;
    double m_sum_y = 0;
    double m_sum_z = 0;

    uint64_t data_count = 0;

    while (1) {
        if (get_accl_data(file, &accl) != 0) {
            return 1;
        }

        if (get_magn_data(file, &magn) != 0) {
            return 1;
        }

        data_count++;

        a_sum_x += accl.xAccl;
        a_sum_y += accl.yAccl;
        a_sum_z += accl.zAccl;

        m_sum_x += magn.xMag;
        m_sum_y += magn.yMag;
        m_sum_z += magn.zMag;

        fprintf(stderr, "Accelerometer: %ld, %ld %ld, avg %lf, %lf, %lf\n",
            accl.xAccl, accl.yAccl, accl.zAccl,
            a_sum_x / (double)data_count,
            a_sum_y / (double)data_count,
            a_sum_z / (double)data_count
            );

#if 0
        fprintf(stderr, "Magnetometer: %ld, %ld, %ld, avg %lf, %lf, %lf\n",
            magn.xMag, magn.yMag, magn.zMag,
            m_sum_x / (double)data_count,
            m_sum_y / (double)data_count,
            m_sum_z / (double)data_count
            );
#endif
    }

    close(file);

    return 0;
}

