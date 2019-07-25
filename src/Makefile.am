bin_PROGRAMS = kal

kal_SOURCES = \
   arfcn_freq.cc \
   c0_detect.cc	 \
   circular_buffer.cc \
   fcch_detector.cc \
   kal.cc \
   offset.cc \
   usrp_source.cc \
   util.cc\
   rsp_dev.cc\
   arfcn_freq.h \
   c0_detect.h \
   circular_buffer.h \
   fcch_detector.h \
   offset.h \
   usrp_complex.h \
   usrp_source.h \
   util.h\
   version.h\
   rsp_dev.h

kal_CXXFLAGS = $(FFTW3_CFLAGS) $(LIBRTLSDR_CFLAGS)
kal_LDADD = $(FFTW3_LIBS) $(LIBRTLSDR_LIBS) -lrt -lgnuradio-sdrplay -lmirsdrapi-rsp -lboost_system -lboost_chrono -lboost_thread
