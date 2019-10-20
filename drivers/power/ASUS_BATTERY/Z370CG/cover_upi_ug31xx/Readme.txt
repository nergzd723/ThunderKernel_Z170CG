Copyright (C) <2015>  uPI semiconductor corp.
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.
    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. <http://www.gnu.org/licenses/>.


Last Update : 2014-06-12, Version 1.00

This document is maintained by Allen Teng <allen.kuoliang.teng@gmail.com> 
as part of the UPI uG31xx gas gauge project.

-------------------------------------------------------------------------------

Steps of porting upi_ug31xx gauge driver

1. Put the "upi_ug31xx" folder at driver/power/battery/

2. Add following lines in Kconfig at driver/power/battery/

    config COVER_GAUGE_UG31XX_Z370CG
      tristate "UPI uG31xx gauge driver"
      depends on I2C
      help
        Say Y here to enable support for UPI uG31xx gauge IC.

3. Add following line in Makefile at driver/power/battery/

    obj-$(CONFIG_COVER_GAUGE_UG31XX_Z370CG) += upi_ug31xx/

4. Check the UPI_BATTERY with menuconfig

