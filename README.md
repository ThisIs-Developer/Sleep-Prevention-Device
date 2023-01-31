# Sleep-Prevention-Device

Feeling sleepy while driving could cause hazardous traffic accident. However, when driving alone on highway or driving over a long period of time, drivers are inclined to feel bored and sleepy.Therefore, we came up with an idea and successfully developed a sleepy detection and alarming system, which could effectively meet this demand.

![205454226-486c2811-a551-45db-bf92-761145e79289](https://user-images.githubusercontent.com/109382325/215797478-0d8448c2-84ac-4cc9-ac9a-365f2e730caf.png)
![205454245-dccd4cf2-45cd-4ca2-9156-2c6a9316d77f](https://user-images.githubusercontent.com/109382325/215797522-f49b6e73-8ea8-4d1f-b141-97d2e724cd01.png)


## Author

- [@ThisIs-Developer](https://github.com/ThisIs-Developer)

## Arduino

Arduino is an open-source physical computing platform based on a simple I/O board and a development environment that implements the Processing/Wiring language. Arduino can be used to develop stand-alone interactive objects or can be connected to software on your computer (e.g. Flash, Processing and MaxMSP). The boards can be assembled by hand or purchased preassembled; the open-source IDE can be downloaded for free at https://arduino.cc

- Tools
  - IDE: [1.x](https://github.com/arduino/Arduino) - [2.x](https://github.com/arduino/arduino-ide)
  - Command line: [CLI](https://github.com/arduino/arduino-cli) - [Lint](https://github.com/arduino/arduino-lint) - [FWUploader](https://github.com/arduino/arduino-fwuploader)
  - Misc: [create-agent](https://github.com/arduino/arduino-create-agent)
- Language specification
  - [Language discussions](https://github.com/arduino/language)
  - [Abstract API specification](https://github.com/arduino/ArduinoCore-API)
  - [Reference docs](https://github.com/arduino/reference-en)
- Cores: [AVR](https://github.com/arduino/ArduinoCore-avr) - [megaAVR](https://github.com/arduino/ArduinoCore-megaavr) - [SAMD](https://github.com/arduino/ArduinoCore-samd) - [SAM](https://github.com/arduino/ArduinoCore-sam) - [Mbed](https://github.com/arduino/ArduinoCore-mbed)
- [Library registry](https://github.com/arduino/library-registry)
- Documentation
  - [docs.arduino.cc sources](https://github.com/arduino/docs-content)
  - [support.arduino.cc sources](https://github.com/arduino/help-center-content)
  - [Built-in examples](https://github.com/arduino/arduino-examples)
- GitHub Actions: [arduino-lint-action](https://github.com/arduino/arduino-lint-action) - [compile-sketches](https://github.com/arduino/compile-sketches) - [report-size-deltas](https://github.com/arduino/report-size-deltas)

## Installation

Detailed instructions for installation in popular operating systems can be found at:

- [Linux](https://www.arduino.cc/en/Guide/Linux) (see also the [Arduino playground](https://playground.arduino.cc/Learning/Linux))
- [macOS](https://www.arduino.cc/en/Guide/macOS)
- [Windows](https://www.arduino.cc/en/Guide/Windows)

## Deployment

Installing Arduino IDE 2.0.3

```bash
https://www.arduino.cc/en/software
```

Arduino IDE 2 Tutorials

```bash
https://docs.arduino.cc/software/ide-v2?_gl=1*1uljego*_ga*MTA4MDY4ODUwNC4xNjc1MTc1OTc2*_ga_NEXN8H46L5*MTY3NTE3NTk3Ni4xLjEuMTY3NTE3NjA4Ni4wLjAuMA..
```

Installing Libraries

```bash
https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries
```

## Environment Variables

To run this project, you will need to add the following environment variables to your .env file

```
#define SSID myssidwhatever
#define PASSWORD mypassword
```

```
#ifdef MYLIB_SLAVE
// slave code
#endif
#ifdef MYLIB_MASTER
//master code
#endif
// common code
```
