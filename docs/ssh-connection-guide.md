# Connecting to the RoboRacer Car over SSH (Windows)

Practical guide and troubleshooting checklist for connecting from this Windows
laptop to the RoboRacer car. Written after a live debugging session on
2026-06-30.

## Connection details

| Item        | Value                  |
| ----------- | ---------------------- |
| User        | `roboracer`            |
| Host (IP)   | `192.168.50.10`        |
| Wi-Fi SSID  | `roboracer`            |
| Password    | `max_verstappen_33`    |

## Quick start

1. Connect this laptop to the **`roboracer`** Wi-Fi network.
2. Open **PowerShell** (or Windows Terminal) and run:

   ```powershell
   ssh roboracer@192.168.50.10
   ```

3. On the **first** connection you'll see a host-key fingerprint prompt —
   type `yes` and press Enter.
4. Enter the password when prompted: `max_verstappen_33`
   (nothing appears as you type — that is normal).

The OpenSSH client ships with Windows 10/11, so no install is needed. Verify
with `ssh -V` (this laptop has `OpenSSH_for_Windows_9.5p2`).

## Troubleshooting checklist

Work through these in order. Each step also shows how to verify it.

### 1. Confirm you are on the car's Wi-Fi

```powershell
(netsh wlan show interfaces) -match 'SSID|State'
```

The `SSID` must read `roboracer` and `State` must be `connected`. If not,
join the `roboracer` network first.

### 2. Confirm the laptop got an address on the car's subnet

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object InterfaceAlias -eq 'WLAN'
```

You should have an IP in `192.168.50.x` (e.g. `192.168.50.129`) with prefix
length `24`. If you instead have a `169.254.x.x` address, the laptop did not
get a DHCP lease — disconnect/reconnect the Wi-Fi or reboot the router.

### 3. Confirm the router/access point is reachable

```powershell
Test-Connection 192.168.50.1 -Count 2
```

The gateway `192.168.50.1` should reply. If it does, the Wi-Fi link itself is
healthy and the problem is the car, not your laptop.

### 4. Confirm the car itself is on the network

```powershell
Test-Connection 192.168.50.10 -Count 2
Get-NetNeighbor 192.168.50.10
```

- If `Test-Connection` succeeds → go to step 5.
- If `Get-NetNeighbor` shows **State `Incomplete`** and a MAC of
  `00-00-00-00-00-00`, the car is **not answering at all**. It is most likely:
  - **powered off** or still booting → power-cycle the car and wait ~60 s;
  - **not connected to its own Wi-Fi** → check the car's onboard computer;
  - **using a different IP** → see the discovery sweep below.

#### Find the car's actual IP (subnet sweep)

If you suspect the car came up on a different address, scan the subnet, then
read the ARP table to see every device that responded:

```powershell
1..254 | ForEach-Object { Start-Job { param($i) Test-Connection "192.168.50.$i" -Count 1 -Quiet } -ArgumentList $_ } | Out-Null
Get-Job | Wait-Job | Out-Null; Get-Job | Remove-Job -Force
Get-NetNeighbor -AddressFamily IPv4 |
  Where-Object { $_.IPAddress -like '192.168.50.*' -and $_.LinkLayerAddress -ne '00-00-00-00-00-00' } |
  Select-Object IPAddress, LinkLayerAddress, State | Sort-Object IPAddress
```

Any IP besides `192.168.50.1` (the router) is a candidate for the car. SSH to
that address instead.

### 5. Confirm the SSH port is open

```powershell
(Test-NetConnection 192.168.50.10 -Port 22).TcpTestSucceeded
```

- `True` → the SSH server is up; `ssh roboracer@192.168.50.10` should work.
- `False` while ping succeeds → the car is on the network but `sshd` is not
  running. Start/enable it on the car (`sudo systemctl start ssh`).

## Other gotchas

- **Multiple network adapters / phone tethering.** This laptop also had an
  iPhone USB-tethering connection (`Ethernet 2`, `172.20.10.x`) active. That
  link carries the default *internet* route but does **not** block traffic to
  the on-link `192.168.50.x` subnet, so SSH still works over Wi-Fi. If you ever
  hit routing oddities, temporarily disable the extra adapter:

  ```powershell
  Disable-NetAdapter -Name 'Ethernet 2' -Confirm:$false   # re-enable with Enable-NetAdapter
  ```

- **Changed host key.** If you reflash the car or it changes identity, SSH
  refuses to connect with a "REMOTE HOST IDENTIFICATION HAS CHANGED" warning.
  Clear the stale key and reconnect:

  ```powershell
  ssh-keygen -R 192.168.50.10
  ```

- **Passwordless login (optional).** To stop typing the password each time,
  copy your public key to the car:

  ```powershell
  # generate a key once (press Enter through the prompts)
  ssh-keygen -t ed25519
  # copy it to the car (creates ~/.ssh/authorized_keys there)
  type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh roboracer@192.168.50.10 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
  ```

- **Shortcut alias (optional).** Add an entry to `C:\Users\Student\.ssh\config`
  so you can just type `ssh roboracer`:

  ```
  Host roboracer
      HostName 192.168.50.10
      User roboracer
  ```

## Today's diagnosis (2026-06-30)

The SSH client, Wi-Fi connection, and credentials were all correct. The laptop
was connected to the `roboracer` Wi-Fi (`192.168.50.129/24`) and the router
`192.168.50.1` responded normally. However, `192.168.50.10` did not answer ping
or ARP (state `Incomplete`), and a full subnet sweep found **only** the router
online. **The car was not on the network** — power it on / connect it to its
Wi-Fi (or find its real IP with the sweep above), then retry the `ssh` command.
