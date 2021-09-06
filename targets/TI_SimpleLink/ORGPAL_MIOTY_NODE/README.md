# ORGPAL MIOTY board

The ORGPAL MIOTY board contains a TI CC1352R1 device.


## Floating point

The current build is set to add support for single-precision floating point.
Meaning that `System.Math` API supports only the `float` overloads. The `double` ones will throw a `NotImplementedException`.

## nanoBooter

Because this CPU has a proprietary boot mechanism it does use nanoBooter.

## Flashing nanoCLR

Follow the instructions [here](http://docs.nanoframework.net/content/ti-cc32xx/flash-nanoclr.html) on how to flash nanoCLR image in the board.
