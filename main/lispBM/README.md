# lispBM (LBM)

LispBM is a lisp or scheme like programming language for
microcontrollers.  LispBM also borrows a couple of ideas from Erlang
when it comes to concurrency, message passing, pattern matching and
process monitoring.  The LispBM runtime system can be compiled for
either 32 or 64 bit platforms and runs on a wide range of hardware
such as for example STM32, NRF52, ESP32 or X86.  When running the
LispBM runtime system on a microcontroller it can be built on top of
ChibiOS, FreeRTOS or ZephyrOS or, if you are adventurous, on bare-metal.
LispBM can also be built to run on top of a regular linux. 

![LispBM mascot](https://github.com/svenssonjoel/lispBM/blob/master/mascot/lispbm_llama_small.png)

The LispBM mascot, Lispy the llama, was created by
[PixiLadyArt](https://www.instagram.com/pixiladyart/). Thank you, Pixi! 

## Want to get involved and help out?

There are lots of interesting things to code on in and around the
LispBM runtime system. I would love to interact with people who are interested
in high-level programming languages on microcontrollers (or elsewhere) so please
do not be shy to make contact. Together we can make great stuff happen ;)

1. Are you interested in microcontrollers and programming languages?
2. You find it fun to mess around in C code with close to zero comments?
3. Then join in the fun. Lots to do, so little time!
4. Poke me by mail bo(dot)joel(dot)svensson(whirly-a)gmail(dot)com.

There are areas where insights and help would matter greatly. Some I can
think of are:

1. Running LispBM on system simulators.
2. LispBM efficiency, compilation, byte-code and real time garbage collection.
3. Code size optimisation. Do more with less, while maintaining performance.
4. Documentation and testing.
5. Useful extension libraries.
6. Lisp scriptable home automation.
7. Lisp scriptable MIDI sequencers.
8. Lisp scriptable Audio synthesizers.
9. ...

## Getting started 

The easiest way to get started with LispBM programming is to use
[VESC-Tool](https://vesc-project.com/vesc_tool) together with the
[VESC EXPRESS Firmware](https://github.com/vedderb/vesc_express) on an [esp32c3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html).

VESC-Tool gives you access to a very capable [IDE](https://vesc-project.com/sites/default/files/lisp%20overview.png) for lisp programming on embedded systems
and the VESC Express firmware provides built in functionality to interface with CAN, WIFI, BLE and UART
as well as GPIO and display drivers.

If you are looking for a more bare-bones way to add scripting capabilities to your
embedded system, LispBM is quite platform independent and can be compiled for many
different MCUs and systems. LispBM can tie into the rest of your application via
what we call "extensions" which are C functions that can be called from your LispBM scripts. 

### Documentation

 - Work in progress [LispBM language reference](./doc/lbmref.md).
 - Work in prograss [LispBM display library reference](./doc/displayref.md).
 - [LispBM runtime system library reference](./doc/runtimeref.md).
 - Gotchas and caveats [Gotchas and caveats](./doc/gotchas.md).
 - !OUTDATED! [LispBM programming manual](./doc/manual).
 - C code documentation can be found [here](http://svenssonjoel.github.io/lbmdoc/html/index.html).
 - LispBM's internals are documented as a series of (now quite outdated) [blog posts](http://svenssonjoel.github.io).
 - There are [demonstrations on YouTube](https://youtube.com/playlist?list=PLtf_3TaqZoDOQqZcB9Yj-R1zS2DWDZ9q9).

## The LBM REPL

There is an example REPL implementation that runs on X86 32 or 64bit in the `repl` directory.
The REPL depends on libreadline.


**REPL Dependencies for 32Bit executable on 64Bit linux:**
* libreadline
* lib32readline
* gcc-multilib
* libpng

On Ubunty you can obtain the depencies by:
`sudo apt-get install gcc-multilib libreadline-dev lib32readline-dev libpng-dev libpng-dev:i386`

Then issue command `make` in the repl directory.

### Editor support

* [vesc_tool](https://vesc-project.com/vesc_tool)
* [vscode support](https://marketplace.visualstudio.com/items?itemName=rasmus-soderhielm.lispbm-language-support)

