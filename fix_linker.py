Import("env")

env["_LIBFLAGS"] =  ('-Wl,--start-group -Wl,--whole-archive '
                    '${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, LIBSUFFIXES, __env__)} '
                    '-Wl,--no-whole-archive -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lmicroros -Wl,--end-group')