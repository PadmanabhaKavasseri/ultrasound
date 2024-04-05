ACLOCAL_AMFLAGS = -I m4

bin_PROGRAMS = tdk-chx01-get-data-app

AM_CPPFLAGS = -I./                \
-I../recipe-sysroot/usr/include \
-I ./invn/common/

tdk_chx01_get_data_app_SOURCES := \
    tdk-chx01-get-data.c

tdk_chx01_get_data_CPPFLAGS = -Wno-all
tdk_chx01_get_data_CPPFLAGS += -Wno-error
tdk_chx01_get_data_CPPFLAGS += -Wno-unused-parameter
tdk_chx01_get_data_CPPFLAGS += -DARM_BASED
tdk_chx01_get_data_CPPFLAGS += -I.
tdk_chx01_get_data_CPPFLAGS += -lm
