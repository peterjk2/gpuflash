# GPUFlash
## Install
Requirements:
```
Python
SCons
```
Build:
```
cd FlashGPU
./build.py --simplessd -j8
```
## SSD parameters
Path:
```
FlashGPU-srccode/src/simplessd/config/readprefetch.cfg
```
Parameters of note (internal flash buffer):
```
CacheSize
CacheWaySize
WriteCacheSize
WriteCacheWaySize
EnableReadCache
EnableWriteCache
```
## Macsim parameters
Path:
```
FlashGPU-srccode/bin/params.in
```
trace_file_list:
```
First line: # of traces to execute
Following lines: Full path to kernel_config.txt
```
kernel_config.txt (in trace folder):
```
newptx
14
-1
Following lines: Full paths to each Trace.txt file (THIS WILL HAVE TO CHANGE FOR EACH MACHINE)
```
## Execution
```
cd bin/
./macsim 
```
## Credits
FlashGPU: https://github.com/FlashGPU/FlashGPU-srccode

Macsim: https://github.com/gthparch/macsim

SimpleSSD: https://github.com/SimpleSSD/SimpleSSD

GPUOcelot (traces): https://github.com/gtcasl/gpuocelot
