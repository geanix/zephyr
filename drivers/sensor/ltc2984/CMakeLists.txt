#
# Copyright (c) 2019 Geanix ApS
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LTC2984 ltc2984.c)
