/*

    USB Thermometer CLI v1 Copyright 2017 jaka

*/

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define TIMEOUT 1
#define DS18X20_SP_SIZE 9
#define DS18X20_GENERATOR 0x8c

typedef unsigned char uchar;

/*
   Signals taken from:
   https://www.maximintegrated.com/en/app-notes/index.mvp/id/214
*/

static uchar lsb_crc8(uchar *data_in, unsigned int len, const uchar generator) {

   uchar i, bit_counter, crc = 0;

   for (i = 0; i < len; i++) {
      crc ^= *(data_in + i);
      bit_counter = 8;
      do {
         if (crc & 0x01)
            crc = (((crc >> 1) & 0x7f) ^ generator);
         else
            crc = (crc >> 1) & 0x7f;
         bit_counter--;
      } while (bit_counter > 0);
   }
   return crc;
}

static int file_exists(char *filename)
{
   struct stat st;

   return (stat(filename, &st) == 0);
}

static int owReset(int fd)
{
   int rv;
   uchar wbuff, rbuff;
   fd_set readset;
   struct timeval timeout_tv;
   struct termios term;

   tcflush(fd, TCIOFLUSH);

   if (tcgetattr(fd, &term) < 0) {
      fprintf(stderr, "Error, could not get baud rate!\n");
      return -1;
   }
   term.c_cflag &= ~CSIZE | CS8;
   cfsetispeed(&term, B9600);
   cfsetospeed(&term, B9600);
   tcsetattr(fd, TCSANOW, &term);

   /* Send the reset pulse. */
   wbuff = 0xf0;
   write(fd, &wbuff, 1);

   timeout_tv.tv_usec = 0;
   timeout_tv.tv_sec = TIMEOUT;

   FD_ZERO(&readset);
   FD_SET(fd, &readset);

   if (select(fd + 1, &readset, NULL, NULL, &timeout_tv) > 0) {

      if (FD_ISSET(fd, &readset)) {
         read(fd, &rbuff, 1);
         switch (rbuff) {
            case 0:
               /* Ground. */
            case 0xf0:
               /* No response. */
               rv = -1; break;
            default:
               /* Got a response */
               rv = 0;
         }
      }
      else {
         rv = -1;
      }
   }
   else
      rv = -1; /* Timed out or interrupt. */

   term.c_cflag &= ~CSIZE | CS6;
   cfsetispeed(&term, B115200);
   cfsetospeed(&term, B115200);

   tcsetattr(fd, TCSANOW, &term);

   return rv;
}

static uchar owWriteByte(int fd, uchar wbuff)
{
   char buf[8];
   uchar rbuff, i;
   size_t remaining, rbytes;
   fd_set readset;
   struct timeval timeout_tv;

   tcflush(fd, TCIOFLUSH);

   for (i = 0; i < 8; i++)
      buf[i] = (wbuff & (1 << (i & 0x7))) ? 0xff : 0x00;
   write(fd, buf, 8);

   timeout_tv.tv_usec = 0;
   timeout_tv.tv_sec = 1;

   FD_ZERO(&readset);
   FD_SET(fd, &readset);

   rbuff = 0;
   remaining = 8;
   while (remaining > 0) {

      if (select(fd + 1, &readset, NULL, NULL, &timeout_tv) > 0) {

         if (FD_ISSET(fd, &readset)) {
            rbytes = read(fd, &buf, remaining);
            for (i = 0; i < rbytes; i++) {
               rbuff >>= 1;
               rbuff |= (buf[i] & 0x01) ? 0x80 : 0x00;
               remaining--;
            }
         }
         else
            return 0xff;
      }
      else
         /* At last timeout will terminate while-loop. */
         return 0xff;
   }
   return rbuff;
}

static uchar owReadByte(int fd)
{
   return owWriteByte(fd, 0xff);
}

void warn_owWriteByte(int fd, uchar wbuff)
{
   if (owWriteByte(fd, wbuff) != wbuff)
      fprintf(stderr, "Warining, not expected sensor response!\n");
}

int serialInit(char *serial_port)
{
   int fd;
   struct termios term;

   if (!file_exists(serial_port)) {
      fprintf(stderr, "Error, serial port '%s' does not exist!\n", serial_port );
      return -1;
   }

   if (access(serial_port, R_OK|W_OK) < 0) {
      fprintf(stderr, "Error, you don't have rw permission to access serial port '%s'!\n", serial_port);
      return -1;
   }

   fd = open(serial_port, O_RDWR);
   if (fd < 0) {
      fprintf(stderr, "Error, failed to open serial port device!\n");
      return -1;
   }

   memset(&term, 0, sizeof(term));

   term.c_cc[VMIN] = 1;
   term.c_cc[VTIME] = 0;
   term.c_cflag |= CS6 | CREAD | HUPCL | CLOCAL;

   cfsetispeed(&term, B115200);
   cfsetospeed(&term, B115200);

   if (tcsetattr(fd, TCSANOW, &term) < 0) {
      close(fd);
      fprintf(stderr, "Error, could not set baud rate!\n");
      return -1;
   }

   tcflush(fd, TCIOFLUSH);

   return fd;
}

int DS18B20_measure(int fd)
{
   if (owReset(fd) < 0) {
      fprintf(stderr, "Error, no sensor found!\n");
      return -1;
   }
   warn_owWriteByte(fd, 0xcc);
   warn_owWriteByte(fd, 0x44);
   return 0;
}

int DS18B20_acquire(int fd)
{
   time_t now;
   struct tm *timeptr;
   char timebuf[40];
   unsigned short T;
   float temperature;
   uchar i, crc, sp_sensor[DS18X20_SP_SIZE];

   if (owReset(fd) < 0) {
      fprintf(stderr, "Error, no sensor found!\n");
      return -1;
   }
   warn_owWriteByte(fd, 0xcc);
   warn_owWriteByte(fd, 0xbe);

   for (i = 0; i < DS18X20_SP_SIZE; i++)
      sp_sensor[i] = owReadByte(fd);

   if ((sp_sensor[4] & 0x9f) != 0x1f) {
      fprintf(stderr, "Error, sensor not found!\n");
      return -1;
   }

   crc = lsb_crc8(&sp_sensor[0], DS18X20_SP_SIZE - 1, 0x8c);
   if (sp_sensor[DS18X20_SP_SIZE - 1] != crc) {
      fprintf(stderr, "Error, sensor CRC mismatch!\n");
      return -1;
   }

   T = (sp_sensor[1] << 8) + (sp_sensor[0] & 0xff);

   if ((T >> 15) & 0x01) {
      T--;
      T ^= 0xffff;
      T *= -1;
   }
   temperature = (float)T / 16;

   time(&now);
   timeptr = localtime(&now);
   strftime(timebuf, sizeof(timebuf), "%b %d %H:%M:%S", timeptr);

   printf("%s Sensor C: %.2f\n", timebuf, temperature);
   return 0;
}

#define OPTS_HELP 1
#define OPTS_VERBOSE (1 << 1)

int main(int argc, char **argv)
{
   char *serial_port = NULL;
   struct timeval wait_tv;
   char c;
   int fd;
   int opts = OPTS_VERBOSE, rv = 0;

   while ((c = getopt(argc, argv, "hqs:")) != -1) {
      switch (c) {
         case 'h':
            opts |= OPTS_HELP;
            break;
         case 'q':
            opts &= ~OPTS_VERBOSE;
            break;
         case 's':
            serial_port = optarg;
            break;
         case '?':
            return -1;
      }
   }

   if (opts & OPTS_HELP)
      opts |= OPTS_VERBOSE;

   if (opts & OPTS_VERBOSE)
      printf("USB Thermometer CLI v1 Copyright 2017 jaka\n");

   if (opts & OPTS_HELP) {
      printf("\t-q\tQuiet mode\n");
      printf("\t-s\tSet serial port\n");
      return 0;
   }

   if (!serial_port)
      serial_port = "/dev/ttyUSB0";
   if (opts & OPTS_VERBOSE)
      printf("Using serial port: %s\n", serial_port);

   fd = serialInit(serial_port);
   if (fd > 0) {

      if (DS18B20_measure(fd) < 0) {
         close(fd);
         return -1;
      }

      wait_tv.tv_usec = 0;
      wait_tv.tv_sec = 1;

      if (opts & OPTS_VERBOSE)
         printf("Waiting for response ...\n");

      select(0, NULL, NULL, NULL, &wait_tv);

      if (DS18B20_acquire(fd) < 0)
         rv = -1;

      close(fd);
      return rv;

   }

   return -1;
}
