# A Value Array Written to a Run of Named Channels Must Be Indexed by Name

The PCA9685 can write a run of consecutive channels in one transaction, so this
firmware batches: six channels for both motors, three for one RGB LED. The
values go in an array, the array is positional, and the position silently means
*a channel number defined in another file*. Nothing connects the two.

That is the whole defect. It is not a crash and it is not a compile error:
renumber the channels in `pin_config.h` and the module keeps compiling, keeps
writing the same count of consecutive channels, and drives the wrong pins.

> **The law: if an array's index means a hardware coordinate, place values by
> that coordinate — `[CH_SLOT(NAME)] = value` — never by argument order. An
> ordering that is only true because two files happen to agree is an
> undeclared invariant, and the compiler will not check it for you.**

## Both instances, found together (2026-09)

| Site | The array | What the position meant |
|---|---|---|
| `motor_controller.c` `set_motors()` | `{r_in1, r_in2, r_pwm, l_in1, l_in2, l_pwm}` | channels 8–13 in per-motor order |
| `led_controller.c` `led_set_hardware()` | `{red, green, blue}` | `LED_*_R/G/B_CHANNEL` being consecutive **and in that order** |

The motor one was live: channels 8–13 were renumbered to follow the
TB6612FNG's control header (PWMA, AIN2, AIN1, BIN1, BIN2, PWMB) so the jumpers
run straight across, and the positional array would have kept writing the old
assignment — a duty cycle onto a direction pin and a direction bit onto PWMA.

The LED one had not bitten yet, and is worse when it does: it fails *silently
and cosmetically*. `activity_trace` holds a red LED to mean "the last camera
capture failed". Swap `LED_LEFT_G_CHANNEL` and `LED_LEFT_B_CHANNEL` and that
hold shows up blue, so the indicator misreports the subsystem it is watching
and the search starts in the camera path.

## The fix, in three parts

1. **Place by coordinate.** C99 designated initialisers make this free:

   ```c
   #define MOTOR_CH_SLOT(ch) ((ch) - MOTOR_FIRST_CHANNEL)

   const uint16_t values[MOTOR_CHANNEL_COUNT] = {
       [MOTOR_CH_SLOT(MOTOR_RIGHT_PWM_CHANNEL)] = r_pwm,
       [MOTOR_CH_SLOT(MOTOR_RIGHT_IN2_CHANNEL)] = r_in2,
       /* … */
   };
   ```

   Once indexed this way, mis-ordering is not a bug you can write.

2. **Static-assert the block property the batch write depends on.** One
   expression proves all three of it — every channel inside the block, no two
   sharing a slot, no holes — and turns the next renumber into a compile error:

   ```c
   _Static_assert(((1u << MOTOR_CH_SLOT(MOTOR_RIGHT_PWM_CHANNEL)) | /* … all six … */) ==
                      ((1u << MOTOR_CHANNEL_COUNT) - 1u),
                  "the six motor channels must be a gap-free block starting at MOTOR_FIRST_CHANNEL");
   ```

3. **Pin the placement in a host test, with values that cannot collide.** The
   assert proves contiguity, not correctness of placement — a swap of two
   channels keeps the block gap-free and passes it. Use distinguishable inputs:
   different speeds per motor and opposite directions; an LED colour with three
   different components and none of them 0 or 255. A probe like `{255, 0, 0}`
   passes with two slots swapped.

`test_each_value_lands_on_its_own_channel` (motors) and
`test_led_controller.c` do this. Both were mutation-checked by restoring the
positional literal; the LED one additionally by swapping G and B in
`pin_config.h`, which the static assert accepts and the test rejects — that
pair is what shows the test earns its place beside the assert.

## Where to look for more

Any batch write whose API takes a *first item* plus a *count*:

```sh
rg -n "set_multi|_write_run|first_ch|start_channel|base_reg" packages/ --glob '!build*'
```

The tell is a call whose first argument is a named constant and whose payload
is a brace-list with no names in it. Same shape appears wherever a register
block, a DMA descriptor set, or a contiguous GPIO group is written at once.

## Related

- `gated-init-paths.md` — the sibling failure: code behind a peripheral gate is
  unrun until the peripheral is fitted, so these arrays were never exercised
  until the PCA9685 arrived
- `build-guide-drift-guard.md` § *A literal is invisible to the guard* — the
  documentation instance of the same law: a channel number typed into a page is
  a coordinate referenced by value rather than by name
- `~/.claude/rules/offload-to-deterministic-substrate.md` — the assert and the
  test are the substrate; "remember to keep these in sync" is not
