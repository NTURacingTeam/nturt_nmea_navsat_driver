/*
 * Copyright (C) 2015 - 2020 QXSI, all rights reserved.
 *
 * This is the demo program for the QXSI PS-SDK.
 * The implementation here is just for reference, please refer to the header file `qxwz_sdk.h`
 * for the detailed definition of the structures and APIs.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include "qxwz_sdk.h"
#include <wiringSerial.h>
#include <wiringPi.h>

//#define DEMO_GGA_STR        "$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B"

static qxwz_uint32_t glob_flag = 0;
static qxwz_uint64_t glob_ts = 0;

static qxwz_uint32_t sdk_auth_flag = 0;
static qxwz_uint32_t sdk_start_flag = 0;
qxwz_sdk_cap_info_t sdk_cap_info = {0};

char gga[128];
int f9p_vcp_fd;
const char *f9p_vcp_dev = "/dev/ttyS0";
int f9p_vcp_baudrate = 460800; 

static int get_gga(char *p_gga); 

static int get_gga(char *p_gga)
{
	int count,count_temp;
	int gngga_count,gngga_count_temp;
	char *p_gngga_start = NULL;
	char *p_gngga_end = NULL;

	if(0 > wiringPiSetup() ){
		return -1;
	}

	for(int i=0;i<10;i++){
		if(0 < (count = serialDataAvail(f9p_vcp_fd))){
			if(256>count){
			//	printf("number %d\n",count);
				continue;
			}
			char *nmea_temp = (char *)malloc(count+1);
			if(NULL != nmea_temp){
				memset(nmea_temp,0,count+1);
				count_temp = count;
				while(count-- > 0){
					*nmea_temp++ = (char)serialGetchar(f9p_vcp_fd);
				}
				nmea_temp = nmea_temp - count_temp;
				//printf("%s",nmea_temp);
				p_gngga_start = strstr(nmea_temp,"$GNGGA");
				if(NULL == p_gngga_start){
					free(nmea_temp);
					nmea_temp = NULL;
					continue;	
				}else{
					gngga_count = 0;
					p_gngga_end = p_gngga_start;
					while('\n' != *p_gngga_end && 100 > gngga_count){
						gngga_count+=1;
						p_gngga_end++;
					}
					//printf("get gngga\n");
					if(100 > gngga_count){
							gngga_count_temp = gngga_count;
							while(gngga_count-- > 0){
								*p_gga++ = *p_gngga_start++;
							}
							*p_gga=0;	
							p_gga = p_gga - gngga_count_temp ;
							printf("%s\n",gga);//LINUX NEED ENTER
							free(nmea_temp);
							nmea_temp = NULL;
							p_gngga_start = NULL;
							p_gngga_end = NULL;

					}else{
						free(nmea_temp);
						nmea_temp = NULL;
						p_gngga_start = NULL;
						p_gngga_end = NULL;
					}
				}
			}
		}
				
	}
	//serialClose(f9p_vcp_fd);
	return 0;
}
static qxwz_void_t demo_show_caps(qxwz_sdk_cap_info_t *cap_info)
{
    qxwz_int32_t loop = 0;

    printf("total capabilities: %d\n", cap_info->caps_num);
    for (loop = 0; loop < cap_info->caps_num; ++loop) {
        printf("idx: %d, cap_id: %u, state: %d, act_method: %d, expire_time: %llu\n",
            loop + 1,
            cap_info->caps[loop].cap_id,
            cap_info->caps[loop].state,
            cap_info->caps[loop].act_method,
            cap_info->caps[loop].expire_time);
    }
}

static qxwz_void_t demo_on_auth(qxwz_int32_t status_code, qxwz_sdk_cap_info_t *cap_info) {
    if (status_code == QXWZ_SDK_STAT_AUTH_SUCC) {
        sdk_auth_flag = 1;
        sdk_cap_info = *cap_info;
        demo_show_caps(cap_info);
    } else {
        printf("auth failed, code=%d\n", status_code);
    }
}

static qxwz_void_t demo_on_start(qxwz_int32_t status_code, qxwz_uint32_t cap_id) {
    printf("on start cap:status_code=%d, cap_id=%d\n", status_code, cap_id);
    sdk_start_flag = 1;
}


static qxwz_void_t demo_on_status(int code)
{
    printf(" on status code: %d\n", code);
}

static qxwz_void_t demo_on_data(qxwz_uint32_t type, const qxwz_void_t *data, qxwz_uint32_t len)
{
    int offset = 0;
    size_t wcnt = 0;
    int total = len;
	unsigned char *p_data = data;
    //printf(" on data: %d, ptr: %p, len: %d\n", type, data, len);
    
	switch (type) {
        case QXWZ_SDK_DATA_TYPE_RAW_NOSR:
            //printf("QXWZ_SDK_DATA_TYPE_RAW_NOSR\n");

			//send rtcm3v3 datato the f9p
			//for(int i = 0;i<len;i++){
			//	printf("%x ",*((unsigned char *)p_data+i));
			//}
			//printf("\n");
			
			for(int i=0;i<len;i++){
				serialPutchar(f9p_vcp_fd,*((unsigned char*)p_data+i));
			}
            break;
        default:
            printf("unknown type: %d\n", type);
    }
}


static void sdk_test()
{
    /*
     * ** WARNING **
     * PLEASE FIRST CONFIRM THAT YOUR ACCOUNT IS AK OR DSK ?!?
     *
     * If your account is AK (usually with prefix `A`, like: `A00012dwejd`), set the `key_type` to `QXWZ_SDK_KEY_TYPE_AK`.
     * Otherwise, if it is DSK (usually with prefix `D`, like: `D0234jdwejd`), set the `key_type` to `QXWZ_SDK_KEY_TYPE_DSK`.
     * ** WARNING **
     */
    qxwz_sdk_config_t sdk_config = {0};
    /* AK or DSK ? Only choose one in the following codes! */
    /*
     * AK
     */
    sdk_config.key_type = QXWZ_SDK_KEY_TYPE_AK,
    strcpy(sdk_config.key, "A4901kjfiucc");
    strcpy(sdk_config.secret, "c3c7ff1615ef51d4");
    /*
     * DSK
     */
    // sdk_config.key_type = QXWZ_SDK_KEY_TYPE_DSK,
    // strcpy(sdk_config.key, "Your DSK");
    // strcpy(sdk_config.secret, "Your DSS");

    /* set device info */
    strcpy(sdk_config.dev_id, "G6300057831");
    strcpy(sdk_config.dev_type, "u-bloxf9");

    /* set callbacks */
    sdk_config.status_cb    = demo_on_status;
    sdk_config.data_cb      = demo_on_data;
    sdk_config.auth_cb      = demo_on_auth;
    sdk_config.start_cb     = demo_on_start;

    int ret = 0;
    unsigned int tick = 0;
    struct timeval tval = {0};
    gettimeofday(&tval, NULL);

    f9p_vcp_fd = open(f9p_vcp_dev, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (f9p_vcp_fd == -1) {
        printf("open :%s failed \n", f9p_vcp_dev);
        goto END;
	}
    /*
     * init sdk
     */
    ret = qxwz_sdk_init(&sdk_config);
    if (ret < 0) {
        printf("sdk init failed\n");
        goto END;
    }

    /*
     * do authentication
     */
    ret = qxwz_sdk_auth();
    if (ret < 0) {
        printf("call sdk auth failed\n");
        goto END;
    }

    while (1) {
        gettimeofday(&tval, NULL);
        usleep(100 * 1000); /* 100ms */
        if ((++tick % 10) == 0) {
			/* get gga*/
			get_gga(gga);
            /* upload GGA */
            //qxwz_sdk_upload_gga(DEMO_GGA_STR, strlen(DEMO_GGA_STR));
            qxwz_sdk_upload_gga(gga, strlen(gga));
        }

        if (sdk_auth_flag > 0) {
            sdk_auth_flag = 0;
            if (sdk_cap_info.caps_num > 0) {
                qxwz_sdk_start(QXWZ_SDK_CAP_ID_NOSR);   /* start NOSR capability */
            }
        }
    }

    qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR);   /* stop NOSR capability */


END:
    while (qxwz_sdk_cleanup() != 0) {
        usleep(100 * 1000);
    }
}

int main(int argc, char *argv[])
{
    sdk_test();
    return 0;
}
