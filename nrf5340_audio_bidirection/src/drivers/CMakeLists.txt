#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	       #${CMAKE_CURRENT_SOURCE_DIR}/cs47l63_comm.c
	       #${CMAKE_CURRENT_SOURCE_DIR}/ina231.c
		   #${CMAKE_CURRENT_SOURCE_DIR}/es8311.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/es8388.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/app_wdt.c
)
