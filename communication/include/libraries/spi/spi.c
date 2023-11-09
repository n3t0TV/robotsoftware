/*
 * Library based from: 
 * 
 * 		https://github.com/WiringPi/WiringPi (WiringPiSPI)
 *
 * wiringPiSPI.c:
 *	Simplified SPI access routines
 *	Copyright (c) 2012-2015 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>


static void pabort(const char *s)
{
	perror(s);
	abort();
}

// The SPI bus parameters
//	Variables as they need to be passed as pointers later on

//static const char       *spiDev0  = "/dev/spidev0.0" ;
//static const char       *spiDev1  = "/dev/spidev0.1" ;
static const uint8_t     spiBPW   = 8 ;
static const uint16_t    spiDelay = 0 ;

static uint32_t    spiSpeeds [2] ;
static int         spiFds [2] ;


/*
 * spiGetFd:
 *	Return the file-descriptor for the given channel
 *********************************************************************************
 */

int spiGetFd (int channel)
{
  return spiFds [channel & 1] ;
}


/*
 * spiDataRW:
 *	Write and Read a block of data over the SPI bus.
 *	Note the data ia being read into the transmit buffer, so will
 *	overwrite it!
 *	This is also a full-duplex operation.
 *********************************************************************************
 */

int spiDataRW (int channel, unsigned char *data, int len)
{
  struct spi_ioc_transfer spi ;

  channel &= 1 ;

// Mentioned in spidev.h but not used in the original kernel documentation
//	test program )-:

  memset (&spi, 0, sizeof (spi)) ;

  spi.tx_buf        = (unsigned long)data ;
  spi.rx_buf        = (unsigned long)data ;
  spi.len           = len ;
  spi.delay_usecs   = spiDelay ;
  spi.speed_hz      = spiSpeeds [channel] ;
  spi.bits_per_word = spiBPW ;

  return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ;
}


/*
 * spiSetupMode:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */

int spiSetupMode (int channel, int speed, int mode)
{
  int fd ;
  int ret = 0;
  char spiDev [32] ;

  mode    &= 3 ;	// Mode is 0, 1, 2 or 3

// Channel can be anything - lets hope for the best
//  channel &= 1 ;	// Channel is 0 or 1

  snprintf (spiDev, 31, "/dev/spidev0.%d", channel) ;

  if ((fd = open (spiDev, O_RDWR)) < 0)
	pabort("can't open device");

  spiSpeeds [channel] = speed ;
  spiFds    [channel] = fd ;

// Set SPI parameters.

  ret = ioctl (fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1)
	pabort("can't set spi mode");
  
  ret = ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW);
  if (ret == -1)
	pabort("can't set bits per word");
  
  ret = ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
	pabort("can't set max speed hz");

  return fd ;
}


/*
 * spiSetup:
 *	Open the SPI device, and set it up, etc. in the default MODE 0
 *********************************************************************************
 */

int spiSetup (int channel, int speed)
{
  return spiSetupMode (channel, speed, 0) ;
}
