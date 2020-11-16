/*
 * GPS_adafruit.c
 *
 *  Created on: 20 de ago de 2019
 *      Author: Camilla
 */

#include "GPS_adafruit.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"

#define MAXLINELENGTH 120 ///< how long are max NMEA lines to parse?
#define FALSE 0
#define TRUE 1
static uint8_t strStartsWith(const char* str, const char* prefix);

volatile char line1[MAXLINELENGTH]; ///< We double buffer: read one line in and leave one for the main program
volatile char line2[MAXLINELENGTH]; ///< Second buffer
volatile uint8_t lineidx=0;         ///< our index into filling the current line
volatile char *currentline;         ///< Pointer to current line buffer
volatile char *lastline;            ///< Pointer to previous line buffer
volatile uint8_t recvdflag;         ///< Received flag
volatile uint8_t inStandbyMode;     ///< In standby flag


/**************************************************************************/
/*!
    @brief Initialization code used by all constructor types
*/
/**************************************************************************/
void common_init(GPS_data* gps) {
  recvdflag   = FALSE;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;

  gps->hour = gps->minute = gps->seconds = gps->year = gps->month = gps->day =
		  gps->fixquality = gps->satellites = 0; // uint8_t
  gps->lat = gps->lon = gps->mag = 0; // char
  gps->fix = FALSE; // boolean
  gps->milliseconds = 0; // uint16_t
  gps->latitude = gps->longitude = gps->geoidheight = gps->altitude =
		  gps->speed = gps->angle = gps->magvariation = gps->HDOP = 0.0; // float
  gps->lastFix = 2000000000L;		// millis() when last fix received
  gps->lastTime = 2000000000L;    // millis() when last time received
  gps->lastDate = 2000000000L;    // millis() when last date received
  gps->recvdTime = 2000000000L;   // millis() when last full sentence received
  gps->error = FALSE;
}


/**************************************************************************/
/*!
    @brief Parse a NMEA string
    @param nmea Pointer to the NMEA string
    @return True if we parsed it, false if it has an invalid checksum or invalid data
*/
/**************************************************************************/
uint8_t parse(char *nmea, GPS_data* gps) {
  // do checksum check

	//common_init(gps);
  // first look if we even have one
  char *ast = strchr(nmea,'*');
  if (ast != NULL) {
    uint16_t sum = parseHex(*(ast+1)) * 16;
    sum += parseHex(*(ast+2));
    // check checksum
    char *p = strchr(nmea,'$');
    if(p == NULL) return FALSE;

    else{
      /*for (char *p1 = p+1; p1 < ast; p1++) {
        sum ^= *p1;
      }
      if (sum != 0)
        // bad checksum :(
		  return FALSE;*/
    }
  } else {
	  return FALSE;
  }
  // look for a few common sentences
  char *p = nmea;

  if (strStartsWith(nmea, "$GPGGA") || strStartsWith(nmea, "$GNGGA")) {
    // found GGA
    // get time
    p = strchr(p, ',')+1;
    parseTime(p, gps);

    // parse out latitude
    p = strchr(p, ',')+1;
    parseLat(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLatDir(p, gps)) return FALSE;

    // parse out longitude
    p = strchr(p, ',')+1;
    parseLon(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLonDir(p, gps)) return FALSE;

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      gps->fixquality = atoi(p);
      if(gps->fixquality > 0){
    	  gps->fix = TRUE;
    	  gps->lastFix = gps->recvdTime;
      } else
    	  gps->fix = FALSE;
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->satellites = atoi(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->HDOP = atof(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->altitude = atof(p);
    }

    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->geoidheight = atof(p);
    }
    return TRUE;
  }

  if (strStartsWith(nmea, "$GPRMC") || strStartsWith(nmea, "$GNRMC")) {
    // found RMC
    // get time
    p = strchr(p, ',')+1;
    parseTime(p, gps);

    // fix or no fix
    p = strchr(p, ',')+1;
    if(!parseFix(p, gps)) return FALSE;

    // parse out latitude
    p = strchr(p, ',')+1;
    parseLat(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLatDir(p, gps)) return FALSE;

    // parse out longitude
    p = strchr(p, ',')+1;
    parseLon(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLonDir(p, gps)) return FALSE;

    // speed
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->speed = atof(p);
    }

    // angle
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    	gps->angle = atof(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      gps->day = fulldate / 10000;
      gps->month = (fulldate % 10000) / 100;
      gps->year = (fulldate % 100);
      gps->lastDate = gps->recvdTime;
    }
    return TRUE;
  }

  if (strStartsWith(nmea, "$GPGLL") || strStartsWith(nmea, "$GNGLL")) {
    // found GLL
    // parse out latitude
    p = strchr(p, ',')+1;
    parseLat(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLatDir(p, gps)) return FALSE;

    // parse out longitude
    p = strchr(p, ',')+1;
    parseLon(p, gps);
    p = strchr(p, ',')+1;
    if(!parseLonDir(p, gps)) return FALSE;

    // get time
    p = strchr(p, ',')+1;
    parseTime(p, gps);

    // fix or no fix
    p = strchr(p, ',')+1;
    if(!parseFix(p, gps)) return FALSE;

    return TRUE;
  }

  	// we dont parse the remaining, yet!
	return FALSE;

}


/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for time
    @param p Pointer to the location of the token in the NMEA string
*/
/**************************************************************************/
void parseTime(char *p, GPS_data* gps){
    // get time
    uint32_t time = atol(p);
    gps->hour = time / 10000;
    gps->minute = (time % 10000) / 100;
    gps->seconds = (time % 100);

    p = strchr(p, '.')+1;
    gps->milliseconds = atoi(p);
    gps->lastTime = gps->recvdTime;
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for latitude angle
    @param p Pointer to the location of the token in the NMEA string
*/
/**************************************************************************/
void parseLat(char *p, GPS_data* gps) {
  long int degree;
  float minutes;
  char degreebuff[10];
  int div = 0;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      gps->latitude_fixed = degree + minutes;
      gps->latitude = degree / 100000 + minutes * 0.000006F;
      div = gps->latitude/100;
      gps->latitudeDegrees = (gps->latitude-100*div)/60.0;
      gps->latitudeDegrees += div;
    }
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for latitude direction
    @param p Pointer to the location of the token in the NMEA string
    @return True if we parsed it, false if it has invalid data
*/
/**************************************************************************/
uint8_t parseLatDir(char *p, GPS_data* gps) {
    if (p[0] == 'S') {
      gps->lat = 'S';
      gps->latitudeDegrees *= -1.0;
      gps->latitude_fixed *= -1;
    } else if (p[0] == 'N') {
    	gps->lat = 'N';
    } else if (p[0] == ',') {
    	gps->lat = 0;
    } else {
      return FALSE;
    }
    return TRUE;
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for longitude angle
    @param p Pointer to the location of the token in the NMEA string
*/
/**************************************************************************/
void parseLon(char *p, GPS_data* gps) {
  int32_t degree;
  long minutes;
  char degreebuff[10];
  int div = 0;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      gps->longitude_fixed = degree + minutes;
      gps->longitude = degree / 100000 + minutes * 0.000006F;
      div = gps->longitude/100;
      gps->longitudeDegrees = (gps->longitude-100*div)/60.0;
      gps->longitudeDegrees += div;
    }
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for longitude direction
    @param p Pointer to the location of the token in the NMEA string
    @return True if we parsed it, false if it has invalid data
*/
/**************************************************************************/
uint8_t parseLonDir(char *p, GPS_data* gps) {
    if (',' != *p)
    {
      if (p[0] == 'W') {
        gps->lon = 'W';
        gps->longitudeDegrees *= -1.0;
        gps->longitude_fixed *= -1;
      } else if (p[0] == 'E') {
        gps->lon = 'E';
      } else if (p[0] == ',') {
        gps->lon = 0;
      } else {
        return FALSE;
      }
    }
    return TRUE;
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for whether there is a fix
    @param p Pointer to the location of the token in the NMEA string
    @return True if we parsed it, false if it has invalid data
*/
/**************************************************************************/
uint8_t parseFix(char *p, GPS_data* gps) {
    if (p[0] == 'A'){
      gps->fix = TRUE;
      gps->lastFix = gps->recvdTime;
      }
    else if (p[0] == 'V')
      gps->fix = FALSE;
    else
      return FALSE;
    return TRUE;
}

/**************************************************************************/
/*!
    @brief Parse a hex character and return the appropriate decimal value
    @param c Hex character, e.g. '0' or 'B'
    @return Integer value of the hex character. Returns 0 if c is not a proper character
*/
/**************************************************************************/
// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

/**************************************************************************/
/*!
    @brief Time in seconds since the last position fix was obtained. Will
    fail by rolling over to zero after one millis() cycle, about 6-1/2 weeks.
    @return float value in seconds since last fix.
*/
/**************************************************************************/
float secondsSinceFix(GPS_data* gps) {
    return (HAL_GetTick() - gps->lastFix) / 1000.;
}

/**************************************************************************/
/*!
    @brief Time in seconds since the last GPS time was obtained. Will fail
    by rolling over to zero after one millis() cycle, about 6-1/2 weeks.
    @return float value in seconds since last GPS time.
*/
/**************************************************************************/
float secondsSinceTime(GPS_data* gps) {
    return (HAL_GetTick() - gps->lastTime) / 1000.;
}

/**************************************************************************/
/*!
    @brief Time in seconds since the last GPS date was obtained. Will fail
    by rolling over to zero after one millis() cycle, about 6-1/2 weeks.
    @return float value in seconds since last GPS date.
*/
/**************************************************************************/
float secondsSinceDate(GPS_data* gps) {
    return (HAL_GetTick() - gps->lastDate) / 1000.;
}



/**************************************************************************/
/*!
    @brief Checks whether a string starts with a specified prefix
    @param str Pointer to a string
    @param prefix Pointer to the prefix
    @return True if str starts with prefix, false otherwise
*/
/**************************************************************************/
static uint8_t strStartsWith(const char* str, const char* prefix)
{
  while (*prefix) {
    if (*prefix++ != *str++)
      return FALSE;
  }
  return TRUE;
}
