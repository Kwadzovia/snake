/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   This file provides code for the configuration and use of a joystick 
  *          Module
  ******************************************************************************
  * @attention
  *
  * Made by Selikem Kwadzovia
  ******************************************************************************
   */

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__
  /* --------------------- Includes --------------------- */
  #include <stdint.h>
  /* --------------------- Functions --------------------- */

  typedef struct
  {
    uint8_t x_axis;
    uint8_t y_axis;
    uint8_t column;
    enum joystick_direction
    {
      JOY_NORTH = 0,
      JOY_SOUTH,
      JOY_EAST,
      JOY_WEST
    } direction;

  } joystick;

  void joystick_init(joystick * joyHandle);
  void joystick_read(joystick * joyHandle);



#endif /* __JOYSTICK_H__ */