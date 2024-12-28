#
# Recording test program
#
PROGRAMS += recording_test

SRC_FILES_recording_test = \
	src/apps/tests/recording_test.cpp

# Add any specific flags needed for recording playback
LDFLAGS_recording_test = $(LDFLAGS_COMMON) 