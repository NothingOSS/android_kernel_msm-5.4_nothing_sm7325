# Makefile for the focaltech touchscreen drivers.


fts_tp-y	+=  focaltech_core.o
fts_tp-y	+=  focaltech_ex_fun.o
fts_tp-y	+=  focaltech_ex_mode.o
fts_tp-y	+=  focaltech_gesture.o
fts_tp-y	+=  focaltech_esdcheck.o
fts_tp-y   +=  focaltech_point_report_check.o
fts_tp-y   +=  focaltech_test/


fts_tp-y	+=  focaltech_flash.o



fts_tp-y   +=  focaltech_spi.o

#fts_tp-y        += focaltech_test/focaltech_test.o \
#                   focaltech_test/supported_ic/focaltech_test_ft3680.o \
#                   focaltech_test/focaltech_test_ini.o

obj-$(CONFIG_TOUCHSCREEN_FTS)        += fts_tp.o 