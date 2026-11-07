# Field network

Status: tested on 2026-11-07 with a laptop and with the Husky on its full patrol route. Router A alone covers the field, so the Husky never roamed. The Orin still has short stalls. The [setup](#husky-setup) below has not been applied yet.

## Network

Both routers broadcast the SSID `PatrolKnight` and share one subnet. Router A runs DHCP and is the gateway. Router B is an access point, connected by a 15 m Ethernet cable from a LAN port on A to the WAN port on B.

| | Router A | Router B |
| --- | --- | --- |
| Address | `192.168.0.1` (gateway) | `192.168.0.2` (management only) |
| 5 GHz | Channel 149, 40 MHz, `9c:53:22:c5:18:66` | Channel 36, 40 MHz, `50:91:e3:0f:d8:86` |
| 2.4 GHz | Channel 1, `9c:53:22:c5:18:67` | Channel 11, `50:91:e3:0f:d8:87` |

Both use WPA2-PSK and advertise 802.11k and 802.11v, but not 802.11r.

The gateway is `192.168.0.1` on either AP, so only the BSSID shows which router a client is on. The client decides when to roam. Router settings cannot make it roam at a chosen point.

## Laptop results

Two outdoor walks between the routers, recorded with `roam_monitor`. The second run used a shorter ARP retry.

| | ARP retry 1000 ms (default) | ARP retry 100 ms |
| --- | --- | --- |
| Roams | 8 | 7 |
| Interruption per roam | 1009-1156 ms | 157-336 ms |
| Radio handover alone | 78-115 ms | 88-144 ms |
| Outages outside roams | 0 | 0 |
| IP or gateway change | None | None |

After a roam, packets waited for ARP to resolve the gateway. The default 1000 ms retry set the length of the interruption.

Roams happen late. The laptop starts scanning only when the signal drops to about -67 dBm, so it roamed at around -70 dBm while the other router's 5 GHz signal was often 20 dB stronger.

These results describe the laptop. The Husky has a different Wi-Fi adapter and may roam at different points.

## Husky state

Checked on 2026-11-07, before any changes:

| Item | Value |
| --- | --- |
| Platform | Orin, Ubuntu 22.04, L4T R36.4.3, `teame@192.168.0.30` |
| Wi-Fi | USB AIC8800D80, `aic8800_fdrv`, interface `wlx7419f81429a9`, one spatial stream |
| Connection | NetworkManager profile `PatrolKnight`, DHCP |
| Power save | On |
| Band | Not locked |
| ARP retry | Not changed |

With power save on, the Orin had 11 stalls of 500-580 ms in 70 s, at -35 dBm and with no packet loss. Plain `ping` shows them too. Laptop to gateway averaged 1.7 ms and laptop to Orin 87 ms, so the stalls are on the Orin side. Power save is the likely cause, but this is not confirmed.

## Husky field test

One patrol of the full field on 2026-11-07, about 7.6 minutes, before the setup below.

| | Result |
| --- | --- |
| Router | Router A 5 GHz for the whole run. No roam, no reconnect |
| Signal | Median -53 dBm, weakest -70 dBm at the far end of the route |
| Packet loss | 0 |
| Gateway latency | Median 1.1-1.4 ms |
| TX rate | Usually 140-290 Mbit/s, 69 Mbit/s at the weakest point |
| Stalls | 41 of 500-580 ms, with no loss. Often exactly 10.4 s apart |

Patrol works on Router A alone. Router B is not needed on this route today, but keep it on as a backup: -70 dBm leaves little margin, and rain, people or the robot's heading can cost another 5-10 dB.

The stalls are the problem to fix. They pause traffic for about half a second every 11 s. Patrol tolerates this, but video and teleoperation will notice. Power save is the likely cause.

It is still unknown when the Orin would roam, or whether it waits until the link drops. This only matters if the route grows beyond Router A's coverage.

## Husky setup

Run once on the Orin with the robot still. The last command reconnects Wi-Fi, so SSH drops for a few seconds.

```bash
sudo tee /etc/sysctl.d/90-roaming.conf <<'EOF'
net.ipv4.neigh.default.retrans_time_ms=100
net.ipv4.neigh.default.mcast_solicit=10
net.ipv4.neigh.wlx7419f81429a9.retrans_time_ms=100
net.ipv4.neigh.wlx7419f81429a9.mcast_solicit=10
EOF
sudo sysctl --system
sudo nmcli connection modify PatrolKnight 802-11-wireless.powersave 2 802-11-wireless.band a
sudo nmcli connection up PatrolKnight
```

The interface name comes from the adapter's MAC address. Update the file if the adapter is replaced.

Check the settings now, and again after a reboot:

```bash
cat /proc/sys/net/ipv4/neigh/wlx7419f81429a9/retrans_time_ms   # 100
iw dev wlx7419f81429a9 get power_save                          # off
nmcli -g 802-11-wireless.band connection show PatrolKnight     # a
```

On Router A, reserve these DHCP addresses so they survive a restart:

| Device | Address | Why |
| --- | --- | --- |
| Orin, `74:19:f8:14:29:a9` | `192.168.0.30` | SSH and monitoring |
| `zenohd` host | `192.168.0.182` | `zenoh.connect` in `config/zenoh_bridge.yaml` |

## Roaming test

The tool is in `patrol/network/roam_monitor` on the operator laptop, not in this repo. It needs no sudo on the Orin and no internet. Run it from the laptop while connected to `PatrolKnight`:

```bash
./robot.sh deploy     # the copy on the Orin is out of date
./robot.sh start
# open http://192.168.0.30:8770/ and drive
./robot.sh stop
./robot.sh fetch      # recordings go to sessions/robot/
```

1. After the setup, park the robot near Router A and record 1-2 minutes. Expect no stalls.
2. Run the full patrol route again and compare with the [field test](#husky-field-test).
3. Repeat with `./launch.sh` running, so the Wi-Fi carries real Zenoh and video traffic.
4. Only if the route grows: drive past Router B until Router A drops below -75 dBm, and check that the Orin roams instead of disconnecting.

Proposed pass criteria. The roam rows apply only if the robot roams:

| Check | Target |
| --- | --- |
| Interruption per roam | Under 500 ms |
| Stalls outside roams | None |
| Reconnects | None, and the IP stays `192.168.0.30` |
| Signal before a roam | Above -75 dBm |
| Zenoh and mission manager | Stay connected through every roam |
| Video | Recovers without a restart |

## To fix

- **Stalls on the Orin (first priority):** apply the [setup](#husky-setup) and check that the stalls are gone.
- **Late roaming on the Husky:** not seen yet, because the robot never left Router A's coverage. If the route grows and the Orin roams too late, add a small service that scans when the signal drops below about -62 dBm, or lower the routers' transmit power. Lower power reduces coverage at the edges.
- **Zenoh and WebRTC during a roam:** not tested. Check how `zenoh_bridge` and the video stream behave through an interruption.
- **Router B TX rate:** the laptop reported a median of 117 Mbit/s on B and 275 Mbit/s on A in the last walk. This reading is unreliable at low traffic, so measure real throughput on both routers.
- **Wi-Fi adapter:** if the stalls continue with power save off, the adapter or its driver is the next suspect. Consider a different adapter.

## Open questions

- Does either router offer 802.11r or a fast roaming option? It could shorten the radio handover, but only if the Orin's driver supports it too.
- Which device is `192.168.0.182` in the field, and does it connect by cable or Wi-Fi?
