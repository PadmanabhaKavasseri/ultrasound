# tdk-chx01-get-data-app

## Usage

This application will automatically detect whether the IIO devices is created.  
If IIO device is not created, it will print a warning and exit.  
Otherwise, it will execute depending on the parameters it is given.

```
-h      : print this help message
-d int  : duration, in seconds (default: 10)
-s int  : number of samples (default: 40)
-f int  : sampling frequency, in Hz (default: 5)
-l str  : output logging full file name (default: /usr/chirp.csv)
-n      : Do not load firmware (default: load firmware)
```

_Note_: If this application is started without any parameter, it will be executed using default parameters.

## Buiding from source on RB5 host

Use provided [build.sh](build.sh) build script that will:
1. Push files on RB5 host
    1. __tdk-chx01-get-data.c__ into /usr
    1. __libInvnAlgoRangeFinder.a__ into /usr/bin
    1. __invn_algo_rangefinder.h__ into /usr
    1. __invn/__ folder into /usr
    1. __firmware/*.bin__ into /usr/share/tdk
1. Compile using gcc

Application __tdk-chx01-get-data-app__ is then available from PATH or in _/usr/local/bin_
