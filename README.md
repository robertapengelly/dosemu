## Disclaimer

  This project is currently in early development and has
  only been tested with a handful of DOS applications
  so far.

## License

  All source code is Public Domain.

## Obtain the source code

  git clone https://git.candlhat.org/dosemu.git

## Building

  BSD:
  
      Make sure you have gcc and gmake installed then run
      gmake -f Makefile.unix.
  
  Linux:
  
      Make sure you have gcc and make installed then run
      make -f Makefile.unix.
  
  macOS:
  
      Make sure you have xcode command line tools installed then run
      make -f Makefile.unix.
  
  Windows:
  
      Make sure you have MinGW installed and its bin directory added
      to your PATH environment variable, then run
      mingw32-make.exe -f Makefile.w32.

## About

  Run DOS applications as if they are native system ones.  If an
  application requires a specific DOS version, you can set the
  DOS_VER environment variable to specify the version reported to
  the application.  You can set it through the command line with:
  
  ### Windows
  
    set DOS_VER=xx.xx
  
  ### Linux, macOS or BSD
  
    export DOS_VER=xx.xx
  
  Currently the only programs that have been tested are:
  
    COMMAND.COM, EDLIN.COM, MORE.COM and TREE.COM
  
  Once you're all set, simply run:
  
    dosemu XX.EXE
  
  or
  
    dosemu XX.COM
  
  As you use the project, if you find applications that don't
  function as they should, or encounter missing interrupt
  handlers or CPU instructions, please feel free to open an
  issue and I'll do my best to add support.
