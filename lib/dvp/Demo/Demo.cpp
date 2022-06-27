#include <stdio.h>
#include <stdint.h>
#include <pthread.h> 
#include <math.h>
#include "DVPCamera.h"

#define TEST_TRIG
#define SOFT_TRIG

void* test(void *p)
{
	dvpStatus status;
	dvpHandle h;
	dvpUint32 i,j,k;
	bool trigMode = false;

	char *name = (char*)p;
	char PicName[64];

	printf("test start, %s\r\n", name);
	k = 0;
	do 
	{
		/* Open the device */
		status = dvpOpenByName(name, OPEN_NORMAL, &h);
		if (status != DVP_STATUS_OK)
		{
			printf("dvpOpenByName failed with err:%d\r\n", status);
			break;
		}

		dvpRegion region;
		double exp;
		float gain;

#ifdef TEST_TRIG
		/* Set trigger mode*/
		status = dvpSetTriggerState(h, true);
		if (status == DVP_STATUS_OK)
		{
			dvpSetTriggerSource(h, TRIGGER_SOURCE_SOFTWARE);
			//dvpSetTriggerJitterFilter(h, 1000);
			//dvpSetTriggerDelay(h, 1000);
			dvpSetTriggerInputType(h, TRIGGER_POS_EDGE);
			dvpSetInputIoFunction(h, INPUT_IO_1, INPUT_FUNCTION_TRIGGER);
			trigMode = true;   
		}
#endif
		uint32_t v;
		uint32_t exposureTime;
		dvpSelectionDescr descr;
		dvpFrame frame;
		void *p;

		status = dvpGetQuickRoiSelDescr(h, &descr);
		if (status != DVP_STATUS_OK)
		{
			break;
		}

		/* Grab frames for every preset ROI */ 
		for (i=0; i<descr.uCount; i++)
		{
			/* Change image size */
			status = dvpSetQuickRoiSel(h, i);
			if (status != DVP_STATUS_OK)
			{
				break;
			}

			/* Start stream */
			status = dvpStart(h);
			if (status != DVP_STATUS_OK)
			{
				break;
			}

			/* Grab frames */
			for (j=0; j<5; j++)
			{
				exposureTime = pow(4,j);//1 4 16 64 256 
				status=dvpSetExposure(h,exposureTime*1000);// us
				if (status != DVP_STATUS_OK)
				{
					break;
				}

				status=dvpHold(h);
				if (status != DVP_STATUS_OK)
				{
					break;
				}
				
				status=dvpRestart(h);
				if (status != DVP_STATUS_OK)
				{
					break;
				}
				printf("SetExposure %d ms OK!\n",exposureTime);

				status = dvpSetAnalogGain(h,1);
				if (status != DVP_STATUS_OK)
				{
					break;
				}


#ifdef SOFT_TRIG
				if (trigMode)
				{
					// trig a frame
					status = dvpTriggerFire(h);
					if (status != DVP_STATUS_OK)
					{
						printf("Fail to trig a frame\r\n");
					}
				}
#endif
				status = dvpGetFrame(h, &frame, &p, 1000);
				if (status != DVP_STATUS_OK)
				{
					if (trigMode)
						continue;
					else
						break;
				}

				/* Show framecount and framerate */
				dvpFrameCount framecount;
				status = dvpGetFrameCount(h, &framecount);
				if(status != DVP_STATUS_OK)
				{
					printf("get framecount failed\n");
				}
				printf("framecount: %d, framerate: %f\n", framecount.uFrameCount, framecount.fFrameRate);

				/* Show frame information */
				printf("%s, frame:%llu, timestamp:%llu, %d*%d, %dbytes, format:%d, userValue:%d\r\n", 
					name,
					frame.uFrameID,
					frame.uTimestamp,
					frame.iWidth,
					frame.iHeight,
					frame.uBytes,
					frame.format,
					frame.userValue);

				/* save picture */
				sprintf(PicName, "pic/test-%s_pic_%d_%d.jpg",name, i,j);
				status = dvpSavePicture(&frame, p, PicName, 90);
				if (status == DVP_STATUS_OK)
				{
					printf("Save to %s OK\r\n", PicName);
				}

				k++;
			}

			/* Stop stream */
			status = dvpStop(h);
			if (status != DVP_STATUS_OK)
			{
				break;
			}
		}

		if (status != DVP_STATUS_OK)
		{
			break;
		}
	}while(0);

	dvpClose(h);

	printf("test quit, %s, status:%d\r\n", name, status);
}


int main(int argc, char *argv[])
{
	printf("start...\r\n");

	dvpUint32 i,j,count;
	dvpCameraInfo info[8];
	dvpStatus status;


	/* Update device list */
	dvpRefresh(&count);
	if (count > 8)
		count = 8;

	for (i = 0; i < count; i++)
	{    
		if(dvpEnum(i, &info[i]) == DVP_STATUS_OK)    
		{        
			printf("Camera FriendlyName : %s\r\n", info[i].FriendlyName);    
		}
	}

	/* No device found */
	if (count == 0)
		return 0;

	pthread_attr_t *threadAttr = new pthread_attr_t[count]; 
	pthread_t *th = new pthread_t[count];
	int r;


	for (j=0; j<count; j++)
	{
		pthread_attr_init(&threadAttr[j]); 
		r = pthread_create(&th[j], &threadAttr[j], test, (void*)info[j].FriendlyName); 
		if (r == 0)
		{
			printf("create thread for %s OK\r\n", info[j].FriendlyName);
		}
		else
		{
			printf("pthread_create failed, returned code:%d\r\n", r);
		}
	}

	for (j=0; j<count; j++)
	{
		if (th[j] != NULL)
		{
			void *s;
			pthread_join(th[j], &s);
		}
	}

	delete []threadAttr;
	delete []th;
	return 0;
}
