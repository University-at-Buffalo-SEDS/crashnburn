./build_release.sh
dfu-util -a 0 -s 0x08000000 -D build/Debug/crashnburn.bin
