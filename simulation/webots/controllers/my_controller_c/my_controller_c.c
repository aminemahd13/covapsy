/*
 * File:          my_controller_c.c
 * Date:          May 23, 2023
 * Description:   Control of the TT-02 car for CoVAPSy simulator
 * Author:        Bruno Larnaudie, Anthony Juton
 * Modifications: August 24, 2023
 */

#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>

/* Macros */
#define TIME_STEP 32
#define TABLE_SIZE 200
#define MAX_SPEED_LIMIT 6.28  // Maximum motor speed

const float* range_data;
unsigned char autoMode = 0;

/* Function Prototypes */
void display_instructions(void);
void set_steering_degrees(float angle_degrees);
void set_speed_m_s(float speed_m_s);
unsigned char handle_keyboard_input(void);
void reverse(void);
 
// Speed in km/h
float speed = 0;
float maxSpeed = 28; // km/h

// Max steering angle in degrees
float maxAngleDegrees = 16; 

int main(int argc, char **argv) 
{
  unsigned int i;
  signed int lidar_data_mm_main[360];
  float angle_degrees, speed_m_s;

  /* Initialize Webots driver */
  wbu_driver_init();
  
  /* Enable Keyboard */
  wb_keyboard_enable(TIME_STEP);
  
  /* Enable Lidar */
  WbDeviceTag lidar = wb_robot_get_device("RpLidarA2");
  wb_lidar_enable(lidar, TIME_STEP);
  
  /* Enable point cloud display on the track */
  wb_lidar_enable_point_cloud(lidar);
 
  display_instructions();
  set_steering_degrees(0);
  set_speed_m_s(0);

  /* Main Simulation Loop */
  while (wbu_driver_step() != -1) 
  {
    float distance;

    /* Read Lidar and process data */
    range_data = wb_lidar_get_range_image(lidar); 
    
    // Front distance
    distance = range_data[0];
    if((distance > 0.0) && (distance < 20.0))
      lidar_data_mm_main[0] = (int)(1000 * distance);
    else 
      lidar_data_mm_main[0] = 0;

    // Process first half (right side)
    for(i = 1; i <= 180 ; i++)
    {
      distance = range_data[180 - i];
      if((distance > 0.0) && (distance < 20.0))
        lidar_data_mm_main[i] = (int)(1000 * distance);
      else 
        lidar_data_mm_main[i] = 0;
    }        

    // Process second half (left side)
    for(i = 181; i <= 360 ; i++)
    {
      distance = range_data[540 - i];
      if((distance > 0.0) && (distance < 20.0))
        lidar_data_mm_main[i] = (int)(1000 * distance);
      else 
        lidar_data_mm_main[i] = 0;
    }

    handle_keyboard_input();     

    if(autoMode)
    {
        /************************************************/
        /* Student Program using:                       */
        /* - table: lidar_data_mm_main                 */
        /* - function: set_steering_degrees(.)         */
        /* - function: set_speed_m_s(...)              */
        /* - function: reverse()                       */
        /************************************************/

        // Angle calculation: distance at 60° - distance at -60°
        angle_degrees = 0.02 * (lidar_data_mm_main[60] - lidar_data_mm_main[300]); 
        set_steering_degrees(angle_degrees);

        speed_m_s = 0.5;
        set_speed_m_s(speed_m_s);
    }
  }

  /* Cleanup Webots resources */
  wbu_driver_cleanup();
  return 0;
}

unsigned char handle_keyboard_input(void)
{
  int key;
  key = wb_keyboard_get_key();
  switch(key)
  {
    case -1:
      break;
          
    case 'n':
    case 'N':
      if (autoMode)
      {
        autoMode = 0;
        printf("-------- Auto Mode Disabled -------\n");
      }
      break;
    
    case 'a':
    case 'A':
      if (!autoMode)
      {
        autoMode = 1;
        printf("------------ Auto Mode Enabled -----------------\n");
      }
      break;
      
    default:
      break; 
  }
  return (unsigned char)key;    
}

void display_instructions()
{
  printf("Click on the 3D view to start\n");
  printf("a for auto mode, n for stop\n");
}

void set_steering_degrees(float angle_degrees)
{
  float angle_rad = 0; 
  if(angle_degrees > maxAngleDegrees)
    angle_degrees = maxAngleDegrees;
  else if(angle_degrees < -maxAngleDegrees)
    angle_degrees = -maxAngleDegrees;   

  // Convert to radians: -angle due to coordinate system orientation
  angle_rad = -angle_degrees * 3.14159 / 180.0; 
  wbu_driver_set_steering_angle(angle_rad);
}

void set_speed_m_s(float speed_m_s)
{
  float speed_kmh;
  speed_kmh = speed_m_s * 3.6;
  if(speed_kmh > maxSpeed)
    speed_kmh = maxSpeed;
  if(speed_kmh < 0)
    speed_kmh = 0;
  wbu_driver_set_cruising_speed(speed_kmh);
}

void reverse(void)
{
  wbu_driver_set_cruising_speed(-1.0);
}