/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

static RPlidarDriver *drv = NULL;

static const size_t NUM_SAMPLES = 8192;

static void idle()
{
	glutPostRedisplay();
}

static void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'c':
        {
            drv->stop();
            drv->stopMotor();
            break;
        }
        case 's':
        {
            drv->startMotor();
            drv->startScan(false, true);
        }
    }
}

static void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
}

static void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// prepare the GL array.
	float points[NUM_SAMPLES * 2];

	float x_min = 1e6f, x_max = - 1e6f;
	float y_min = 1e6f, y_max = - 1e6f;

    // fetch result.
    rplidar_response_measurement_node_hq_t nodes[NUM_SAMPLES];
    size_t   count = _countof(nodes);

    u_result op_result = drv->grabScanDataHq(nodes, count, 0);

    if (IS_OK(op_result)) {
        drv->ascendScanData(nodes, count);
        for (int pos = 0; pos < (int)count ; ++pos) {
            float angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
            float distance = nodes[pos].dist_mm_q2 / (1 << 2);

			// convert tht theta and distance to xy.
			float rad = angle * M_PI / 180.0;
			float x = distance * cosf(rad);
			float y = distance * sinf(rad);

            printf("Angle: %03.3f %03.3f XY: %06.3f  %06.3f\n", angle, distance, x, y);
			
			int off = pos * 2;
			points[off] = x;
			points[off + 1] = y;

			// calculate bounding box.
			x_min = (x < x_min ? x : x_min);
			x_max = (x > x_max ? x : x_max);

			y_min = (y < y_min ? y : y_min);
			y_max = (y > y_max ? y : y_max);
        }

		// draw the points.
		glEnableClientState(GL_VERTEX_ARRAY);

		glVertexPointer(2, GL_FLOAT, 0, points);

        glColor3f(1, 1, 1);
		glDrawArrays(GL_POINTS, 0, count);

		glDisableClientState(GL_VERTEX_ARRAY);

        // draw the circles.
        glBegin(GL_LINE_LOOP);
        for(int i = 0; i < 300; i ++)
        {
            double angle = 2.0 * M_PI * i / 300;
            double x = cos(angle) * 200;
            double y = sin(angle) * 200;
            glColor3f(1, 0, 0);
            glVertex2d(x, y);
        }
        glEnd();

        glBegin(GL_LINE_LOOP);
        for(int i = 0; i < 300; i ++)
        {
            double angle = 2.0 * M_PI * i / 300;
            double x = cos(angle) * 300;
            double y = sin(angle) * 300;
            glColor3f(1, 1, 0);
            glVertex2d(x, y);
        }
        glEnd();

        // draw rectangle as robot body.
        glBegin(GL_LINE_LOOP);
        glColor3f(0, 0, 1);
        glVertex2f(-250, -200);
        glVertex2f( 250, -200);
        glVertex2f( 250,  200);
        glVertex2f(-250,  200);
        glEnd();
    }

	glutSwapBuffers();
}

static void close()
{
    // Stop the motor.
    drv->stop();
    drv->stopMotor();

    // Dispose Drive.
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

int main(int argc, char * argv[]) {
	// prepare GLUT.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(512, 512);
    glutCreateWindow("ultra_simple");

	glutDisplayFunc(render);
    glutReshapeFunc(reshape);
	glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutCloseFunc(close);

	glPointSize(1);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    gluOrtho2D(-500, 500, -500, 500);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glRotatef(90, 0, 0, 1);
    glScalef(-1, 1, 1);

	glViewport(0, 0, 512, 512);

	// prepare serial port.
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#elif __APPLE__
        opt_com_path = "/dev/tty.SLAB_USBtoUART";
#else
        opt_com_path = "/dev/ttyUSB1";
#endif
    }

    // create the driver instance
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(int i = baudRateArraySize - 1; i >= 0; -- i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    printf("Using %d baudrate.\n", baudrateArray[i]);
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
		exit(1);
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
		exit(1);
    }

	// enter the GL loop.
	glutMainLoop();
    return 0;
}

