/*
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#define AKM_TRACE 1
#define AKMD_SEQUENTIAL_MEASURE	1
#define AKMD_SEQUENTIAL_TIMER	0
#define AKMD_INTERVAL			100
#define AKMD_BUF_LENGTH			100
#define AKMD_WATERMARK			2

#define TO_STRING_(x) #x
#define TO_STRING(x) TO_STRING_(x)

#define EVENT_CODE_MAGV_X MSC_RX
#define EVENT_CODE_MAGV_Y MSC_RY
#define EVENT_CODE_MAGV_Z MSC_RZ
#define EVENT_CODE_ORIENT_STATUS MSC_ST2

#define DEBUG
#ifdef DEBUG
#define DEBUG_OUTPUT printf
#else
#define DEBUG_OUTPUT
#endif

#if AKM_TRACE
#define FUNCTION_TRACE	printf("%s\n", __func__)
#else
#define FUNCTION_TRACE
#endif


enum {
	buf_enable = 0,
	op_mode,
	/* info, */
	frequency,
	reset,
	selftest,
	watermark,
	sensor_drive,
	in_magn_x_raw,
	in_magn_y_raw,
	in_magn_z_raw,
	element_x_en,
	element_y_en,
	element_z_en,
	num_of_attr,
};

#define IIO_DEVICE_NAME iio:device0
char const * const SYSFS_BUF_ENABLE_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/buffer/enable );
char const * const SYSFS_MODE_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/operation_mode );
char const * const SYSFS_FREQUENCY_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/in_magn_sampling_frequency );
char const * const SYSFS_RESET_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/reset );
char const * const SYSFS_SELFTEST_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/selftest );
char const * const SYSFS_WATERMARK_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/watermark );
char const * const SYSFS_MT_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/sensor_drive );
char const * const SYSFS_MAG_X_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/in_magn_x_raw );
char const * const SYSFS_MAG_Y_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/in_magn_y_raw );
char const * const SYSFS_MAG_Z_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/in_magn_z_raw );
char const * const SYSFS_EBL_X_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/scan_elements/in_magn_x_en );
char const * const SYSFS_EBL_Y_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/scan_elements/in_magn_y_en );
char const * const SYSFS_EBL_Z_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/IIO_DEVICE_NAME/scan_elements/in_magn_z_en );


static volatile bool is_prompt_request = false;

static volatile int g_fds[num_of_attr] = {0};

static int data_fd_list[3];
static int iio_dev_fd = 0;

void signal_handler( int sig )
{
	if ( sig == SIGINT ) {
		is_prompt_request = true;
	}
}

int perform_enable( int arg_en )
{
	char val_en[1];
	int err;

	if(g_fds[arg_en]){
		switch(arg_en){
			case buf_enable:
				g_fds[arg_en] = open(SYSFS_BUF_ENABLE_FILE_NAME, O_RDWR);
				break;
			case element_x_en:
				g_fds[arg_en] = open(SYSFS_EBL_X_EN_FILE_NAME, O_RDWR);
				break;
			case element_y_en:
				g_fds[arg_en] = open(SYSFS_EBL_Y_EN_FILE_NAME, O_RDWR);
				break;
			case element_z_en:
				g_fds[arg_en] = open(SYSFS_EBL_Z_EN_FILE_NAME, O_RDWR);
				break;
			default:
				break;
		}
	}

	if (g_fds[arg_en] < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open set fds[%d] - enable - errno = %d\n", arg_en, err);
		return err;
	}

	val_en[0] = '1';

	if (write(g_fds[arg_en], val_en, sizeof(val_en)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set fds[%d]- enable - errno = %d\n", arg_en, err);
		return err;
	}

	usleep(100);
	return 0;
}

int perform_watermark( int arg_wm )
{
	int err;
	char val_string[10];

	if (g_fds[watermark] < 0) {
		g_fds[watermark] = open(SYSFS_WATERMARK_FILE_NAME, O_RDWR);
		if (g_fds[watermark] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open set - watermark - errno = %d\n", err);
			return err;
		}
	}

	memset(val_string, 0, sizeof(val_string));
	snprintf(val_string, sizeof(val_string), "%d", arg_wm);

	if (write(g_fds[watermark], val_string, sizeof(val_string)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set-watermark - errno = %d\n", err);
		return err;
	}

	return 0;
}

int get_mag_data(int data[4])
{
	char buf[16];
	char *endptr;
	ssize_t nbyte;
	int fd;
	int err = 0;

	FUNCTION_TRACE;

	/* Read X */
	fd = open(SYSFS_MAG_X_RAW_FILE_NAME, O_RDONLY);

	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_x - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read X error: errno = %d\n", err);
		} else {
			data[0] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	/* Read Y */
	fd = open(SYSFS_MAG_Y_RAW_FILE_NAME, O_RDONLY);
	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_y - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read Y error: errno = %d\n", err);
		} else {
			data[1] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	/* Read Z */
	fd = open(SYSFS_MAG_Z_RAW_FILE_NAME, O_RDONLY);
	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_z - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read Z error: errno = %d\n", err);
		} else {
			data[2] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	return 0;
}
int get_trigger_data(void)
{
	int i = 0;
	int maxfdp = 0;
	int ret = 0;
	fd_set readfd;
	struct timeval timeout = {1, 0};
	ssize_t nbyte;
	char buf[16] = {0};
	int err = 0;
	int readvalue = 0;

	/* X */
	data_fd_list[0] = open(SYSFS_MAG_X_RAW_FILE_NAME, O_RDONLY);
	/* Y */
	data_fd_list[1] = open(SYSFS_MAG_Y_RAW_FILE_NAME, O_RDONLY);
	/* Z */
	data_fd_list[2] = open(SYSFS_MAG_Z_RAW_FILE_NAME, O_RDONLY);
	fprintf(stderr, "ecompass: open fd list:%d,%d,%d\n", data_fd_list[0],data_fd_list[1],data_fd_list[2]);

	FD_ZERO(&readfd);
	for (i=0; i<3; i++) {
		FD_SET(data_fd_list[i], &readfd);
		if (i<2) {
			maxfdp = data_fd_list[i] > data_fd_list[i+1]? data_fd_list[i]+1:data_fd_list[i+1];
		}
	}
	ret = select(maxfdp+1, &readfd, NULL, NULL, &timeout);
	switch(ret) {
		case -1:
			fprintf(stderr, "ecompass: select return error\n");
			break;
		case 0:
			fprintf(stderr, "ecompass: select timeout\n");
			break;
		default:
			fprintf(stderr, "ecompass: select success\n");
			for (i=0; i<3; i++) {
				if (FD_ISSET(data_fd_list[i], &readfd)) {
					memset(buf, 0, 16);
					nbyte = read(data_fd_list[i], buf, sizeof(buf));
					close(data_fd_list[i]);
					data_fd_list[i] = 0;
					if (nbyte == 0) {
						err = errno;
						fprintf(stderr, "Read data_fd_list[%d] error: errno = %d\n", i, err);
					} else {
						readvalue = strtol(buf, NULL, 10);
						fprintf(stderr, "ecompass: read data_fd_list[%d] = %d\n", i, readvalue);
					}
				}
			}
			break;
	}
		
	

}

int perform_singleshot( void )
{
	int err;
	char value[1];

	FUNCTION_TRACE;

	if (g_fds[op_mode] < 0) {
		g_fds[op_mode] = open(SYSFS_MODE_FILE_NAME, O_WRONLY);
		if (g_fds[op_mode] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open single-shot - errno = %d\n", err);
			return err;
		}
	}

	value[0] = '1';

	if (write(g_fds[op_mode], value, sizeof(value)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform single-shot - errno = %d\n", err);
		return err;
	}

	return 0;
}

int perform_selftest( void )
{
	int err;
	char value[1];

	FUNCTION_TRACE;

	if (g_fds[selftest] < 0) {
		g_fds[selftest] = open(SYSFS_SELFTEST_FILE_NAME, O_WRONLY);
		if (g_fds[selftest] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open self-test - errno = %d\n", err);
			return err;
		}
	}

	value[0] = '1';

	if (write(g_fds[selftest], value, strlen(value)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform self-test - errno = %d\n", err);
		return err;
	}

	return 0;
}

int perform_interval( int arg_interval )
{
	int err;
	char val_string[10];

	FUNCTION_TRACE;
	printf("perform_interval interval=%d\n",arg_interval);

	if (g_fds[frequency] < 0) {
		g_fds[frequency] = open(SYSFS_FREQUENCY_FILE_NAME, O_RDWR);
		if (g_fds[frequency] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open set - frequency - errno = %d\n", err);
			return err;
		}
	}

	if (arg_interval > 999999999) {
		arg_interval = 999999999;
	}
	if (arg_interval < 0) {
		arg_interval = 0;
	}
	memset(val_string, 0, sizeof(val_string));
	snprintf(val_string, sizeof(val_string), "%d", arg_interval);

	if (write(g_fds[frequency], val_string, sizeof(val_string)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set-frequency - errno = %d\n", err);
		return err;
	}

	return 0;
}

int action_loop(void)
{
	bool is_loop_continue = true;
	int err;
	int data[4];
	char msg[20];
	int i = 0;


	FUNCTION_TRACE;

	/* Init */
	data[0] = data[1] = data[2] = data[3] = 0;

	err = perform_enable(element_x_en);
	err = perform_enable(element_y_en);
	err = perform_enable(element_z_en);
	err = perform_enable(buf_enable);

#if AKMD_SEQUENTIAL_MEASURE
#if AKMD_SEQUENTIAL_TIMER
	err = perform_watermark(AKMD_WATERMARK);
#endif
	err = perform_interval(AKMD_INTERVAL);


	if (err < 0) {
		return err;
	}
	
	while( is_loop_continue ) {
		usleep(1000*100);
		get_trigger_data();
		if( true == is_prompt_request ){
			/* Reset flag */
			is_prompt_request = false;
			/* Menu */
			printf("1) Menu 1.\n");
			printf("2) Menu 2.\n");
			printf("3) Menu 3.\n");
			printf("q) quit.\n");
			fgets(msg, 20, stdin);
			/* Process */
			switch(msg[0]){
				case '1':
					break;
				case '2':
					break;
				case '3':
					break;
				case 'q':
					is_loop_continue = false;
					err = perform_interval(0);
					for (i=0; i<sizeof(g_fds)/sizeof(int); i++) {
						if (g_fds[i] != 0) {
							close(g_fds[i]);
							g_fds[i] = 0;
						}
					}
					break;
				default:
					break;
			}
		}
	}
#else
	while( is_loop_continue ) {

		/* single-shot */
		err = perform_singleshot();
		if (err < 0) {
			return err;
		}
		usleep(AKMD_INTERVAL * 1000);

		/* Get data */
		if (get_mag_data(data)) {
			fprintf(stderr, "get_mag_data failed.\n");
		} else {
			printf("Data:%d, %d, %d\n",
					data[0],
					data[1],
					data[2]);
		}
		if( true == is_prompt_request ){
			/* Reset flag */
			is_prompt_request = false;
			/* Menu */
			printf("1) Menu 1.\n");
			printf("2) Menu 2.\n");
			printf("3) Menu 3.\n");
			printf("q) quit.\n");
			fgets(msg, 20, stdin);
			/* Process */
			switch(msg[0]){
				case '1':
					break;
				case '2':
					break;
				case '3':
					break;
				case 'q':
					is_loop_continue = false;
					break;
				default:
					break;
			}
		}
	}

#endif


	/* Stop measure */
#if AKMD_SEQUENTIAL_MEASURE
	err = perform_interval(0);
	if (err < 0) {
		return err;
	}
#endif

	printf( "ecompass: good bye.\n" );
	return 0;
}

int main( int argc, char** argv )
{
	int err = 0;
	int i;

	/* Init */
	for (i=0; i<num_of_attr; i++) {
		g_fds[i] = -1;
	}

	/* SIGNAL */
	signal( SIGINT, signal_handler );

	/* Main function */
	err = action_loop();

	if (err < 0) {
		fprintf( stderr, "ecompass: error = %d\n", err);
	}

	for (i = 0; i < num_of_attr; i++) {
		if (g_fds[i] >= 0) {
			for (i = 0; i < 100; ++i) {
				if (0 == close(g_fds[i])) {
					break;
				}
				usleep(1000);
			}
		}
	}
	return err;
}
