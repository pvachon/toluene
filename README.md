# Toluene Sniffer

This is the firmware for the BLE sniffer

# To Build

Install the ESP-IDF for ESP32 (see https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html), then simply invoke `make`.

There are a few bugs Toluene exposes in the ESP-IDF that aren't yet upstream. Apply the patch `esp-idf-fixes-for-toluene-sensor.patch` to your ESP-IDF source directory, i.e. using `patch -p1`. This ESP-IDF instance shoud only be used for Toluene, because it will change some expected behaviours for normal GATT client use cases.

# Device Provisioning Process

1. Generate a provisioning key (an EC key on the P-256 curve) - convert public and private key to PEM form
2. Overwrite the public key in `components/identity/include/trusted_pk.h` with your new public key
3. Generate your device CA root key (an EC key on the P-256 curve)
4. Generate your server CA root key (an EC key on the P-256 curve)
5. Generate a self-signed cert for the device CA and server CA respectively. Convert server CA cert to DER format.
6. Build the firmware and flash it to your devices.
7. Invoke `components/identity/identigen/identity.py` with the appropriate arguments. For example:
```
  ./identity.py 
      -E "yourEssidHere" -P password -H "host.that.can.eat.protobufs" -p 24601
      -K provisioning_key_pk.pem -U /dev/ttyUSB0 -C device_ca/device_ca_key.pem 
      -c device_ca/device_ca.crt -L 30 -B 115200 -S server_ca/server_ca.der -I [integer device ID]
```
And you're done!

Note that if you invoke `identity.py` with the `-h` flag it will give you insight into the format for each argument.

# License

This code is licensed under the GNU General Public License v3 or later. See `COPYING` in the root of this source repository for more info.
