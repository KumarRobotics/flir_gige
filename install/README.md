# eBus SDK for Linux
The following content is a summary from the eBus SDK linux quick start guide.  
The original document can be downloaded from [eBus SDK Documentation & Downloads](http://www.pleora.com/support-center/documentation-and-downloads/79).

## Installing the eBus SDK for Linux
There are several steps to complete for installing the eBus SDK for Linux and
optimizing its performance with GigE vision devices.  
Here we integrate all necessary steps into a shell script. For detailed
information, refer to the official documentation.  
To install, simply do:
```bash
sudo ./install.sh
```

## Verify installation
To verify installation, power on and connect your Gige vision device to the
same subnet as your station.  
Then run the following command:
```bash
cd /opt/pleora/ebus_sdk/Ubuntu-12.04-x86_64/bin
./eBUSPlayer
```
Normally, a GUI will show up and it's fairly intuitive for you to connect to
the camera and stream images.

## A note on Ubuntu
The eBus SDK for Linux supports Ubuntu 12.04. It links to `libudev.so.0` in
`/usr/lib` by default.  
The `install.sh` will check your Ubuntu version and link the correct shared
library for you.
