# ODROID-M2 MOSFET trigger research

Reviewed 2026-09-27, without access to the boat. This establishes software pin
identity, not that the connected module accepts the board's output voltage.

## Confirmed mapping

- Use **J2 (40-pin header), physical pin 11**, not GPIO line 11 or WiringPi pin
  11. Hardkernel's `phyToGpio` table maps it to **GPIO3_D4**, native number 124:
  bank 3, bank-relative offset **28**. The same table identifies physical pin
  **6 as ground**. [Hardkernel WiringPi source][wiringpi]
- Hardkernel's board device tree independently names GPIO3_D4 `PIN_11`.
  [Hardkernel board DTS][board]
- Upstream RK3588 describes bank 3 as `gpio@fec40000`, with 32 lines. The
  Rockchip GPIO driver assigns its label from the pinctrl bank name, `gpio3`.
  A `/dev/gpiochipN` suffix is a device identifier, not proof of the bank:
  select/verify bank identity rather than assuming suffix 3 means bank 3.
  [SoC DTS][soc], [GPIO driver][driver], [bank definitions][banks]
- A GPIO request owns the line until released. A successful request/readback
  does not establish voltage at the physical connector or operation of the
  external MOSFET module. [Linux GPIO character-device API][api]

## Electrical and deployment checks

The prior GPIO3_D4/offset-28 assignment is supported by primary sources; there
is no evidence that changing to another header pin would fix the symptom.
Possible causes still include GPIO-chip enumeration, persisted configuration,
pin multiplexing, missing common ground, a module requiring a higher trigger
voltage, or external power/wiring. Hardkernel's board DTS also defines this pin
as a possible MCP251x interrupt pin; check for an enabled CAN expansion overlay
or another line consumer before using it as an output.

Connect pin 11 only to a **compatible logic trigger input**, and pin 6 to the
module's signal ground (for a non-isolated input). Do not power a load from the
GPIO. Treat it as low-voltage logic, not a 5 V supply or 5 V-tolerant input.
The official wiki/schematic endpoints returned an access challenge during this
review, so an exact electrical-high specification was **not independently
verified** here. Measure pin 11 relative to pin 6 with the load safely isolated,
and compare the measured high voltage with the module's documented input-high
threshold. Do not assume a generic high-trigger module accepts 3.3 V. Use an
appropriate level interface if required; never feed its higher voltage back
into the GPIO. A suitable external inactive-state pull-down is needed if the
module does not already provide one: releasing a GPIO is not a guaranteed
electrical low across shutdown/reboot.

Do not infer header orientation from a Raspberry Pi photograph: locate J2 pin
1 using the ODROID-M2 board marking/documentation before counting to pin 11.

[wiringpi]: https://github.com/hardkernel/wiringPi/blob/master/wiringPi/odroidm2.c
[board]: https://github.com/hardkernel/linux/blob/rk35_14.0.0_master/arch/arm64/boot/dts/rockchip/rk3588s-odroid-m2.dtsi
[soc]: https://github.com/torvalds/linux/blob/master/arch/arm64/boot/dts/rockchip/rk3588-base.dtsi
[driver]: https://github.com/torvalds/linux/blob/master/drivers/gpio/gpio-rockchip.c
[banks]: https://github.com/torvalds/linux/blob/master/drivers/pinctrl/pinctrl-rockchip.c
[api]: https://docs.kernel.org/userspace-api/gpio/chardev.html
