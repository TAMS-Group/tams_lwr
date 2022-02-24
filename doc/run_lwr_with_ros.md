# Run KUKA LWR with ROS

## Quick start note
Proceeding to run a demo (In case of error, use the complete start note):
1. Select the appropriate tool `4` and base `2` frame with KRC unit
1. Once the KUKA LWR4+ is up and running:
1. Switch to automatic mode on KRC unit
1. Select code to run in KRC unit
1. Release motor breaks with KRC unit
1. Run `roslaunch tams_lwr ros_fri.launch`
    - In case of error, acknowledge with KRC unit and proceed with 4.
    - Check if appropriate tool and base frame set
    - If success, you will see the following print out:
    ```
    [ INFO] [1624095025.084690764]: ros_fri: iteration 1 state 0
    [ INFO] [1624095025.084901195]: ros_fri: iteration 2 state 0
    [ INFO] [1624095025.089392390]: ros_fri: iteration 3 state 0
    [ INFO] [1624095025.099509523]: ros_fri: iteration 4 state 0
    [ INFO] [1624095025.109566996]: ros_fri: iteration 5 state 0
    [ INFO] [1624095025.119400841]: ros_fri: iteration 6 state 0
    [ INFO] [1624095025.129500956]: ros_fri: iteration 7 state 0
    [ INFO] [1624095025.139523725]: ros_fri: iteration 8 state 0
    [ INFO] [1624095025.149383478]: ros_fri: iteration 9 state 0
    ```
    - If not, run the command above again
1. Run `roslaunch tams_lwr bringup.launch launch_lwr:=false`


## Complete start note

### KUKA LWR
- Presents ROS nodes for real-time (online) motion control.
- For the KUKA LWR-4+ via FRI and the ReflexxesTypeII motion library.
- KUKA LWR setup and device permissions
- Please boot a low-latency kernel for better realtime behaviour.
- The ros_fri interface is configured to run at 100Hz; with the host PC at `192.168.0.2`, the LWR at `192.168.0.1`, and the Schunk gripper at `192.168.0.20`.

- Copy files:
    ```bash
    sudo mkdir -p /opt/FRILibrary/etc
    roscd tams_lwr/config/
    sudo cp 980039-FRI-Driver.init /opt/FRILibrary/etc
    sudo cp limits.conf /etc/security/
    sudo cp 70-persistent-net.rules /etc/udev/rules.d/
    sudo cp interfaces /etc/network/
    ```

- Change default network name (ens33) to old “eth0” on Ubuntu 18.04
    - Edit file:
    ```bash
    sudo vim /etc/default/grub
    ```
    Look for `GRUB_CMDLINE_LINUX` and add the following `net.ifnames=0 biosdevname=0`.
    From:
    ```bash
    GRUB_CMDLINE_LINUX=""
    ```
    To:
    ```bash
    GRUB_CMDLINE_LINUX="net.ifnames=0 biosdevname=0"
    ```
    - Generate a new grub file using the following command.
    ```bash
    sudo grub-mkconfig -o /boot/grub/grub.cfg
    # result:
    # Generating grub configuration file ...
    # Warning: Setting GRUB_TIMEOUT to a non-zero value when GRUB_HIDDEN_TIMEOUT is set is no longer supported.
    # Found linux image: /boot/vmlinuz-4.4.0-15-generic
    # Found initrd image: /boot/initrd.img-4.4.0-15-generic
    # Found memtest86+ image: /memtest86+.elf
    # Found memtest86+ image: /memtest86+.bin
    # done
    ```
    - Reboot

- Optionally, try setting scheduling policy to `SCHED_RR`, to set policy to `SCHED_RR` scheduling with 20 priority:
    ```bash
    chrt -r -p [1..99] {pid}
    chrt -r -p 20 1024
    ```

- Starting the LWR with `ros_fri`
    1. Power the robot up, wait until booted
    2. Check/set the correct robot tool: `KRC2 -> Configure -> Set tool/base -> 4  (Schunk)-> Frame -> 2 (Table)`
    3. Sometimes, the robot will not start in candle position, so: `KRC2 -> Mode T1 -> Joint Level Motion -> move A1 .. A6`
    4. `KRC2 -> Mode C -> Navigator -> FRIControl -> Anwählen Starttaste (I)` wait a few seconds
    5. press green run button -> wait a few seconds, press green run button again
    6. if startup fails (*e.g.* FRI bad communication quality), try to repeat. It might help to stop the motors (O), `Navigator->FRIControl -> Abwählen`, wait a few seconds, then repeat step 4) above.
    7. When `ros_fri/FRILibrary` crashes, FRI might still be active, but in inconsistent state. Reboot the robot.

### Schunk WSG-50 driver:
- The gripper is running the newest firmware version available for the hardware. This still has issues with lua memory usage and garbage collection, and reading the gripper data is slow.
- The current ROS node runs about as fast as possible with our hardware; there are still some errors when reading the tactile sensors.
