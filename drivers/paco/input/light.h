/* 
  Light Sensor Driver

  This is the driver code for  Light sensor
  Copyiiright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/

#ifndef _LIGHT_H /* Double definition is prevented */
#define _LIGHT_H

/*to start light sensor mechanisms */
int illumi_sensor_open (void);

/*to read the light sensor value and fill it in the out parameter*/
int illumi_sensor_read(int *buf);

/*to stop light sensor when the job is done*/
int illumi_sensor_close (void);
#endif /*_LIGHT_H*/
