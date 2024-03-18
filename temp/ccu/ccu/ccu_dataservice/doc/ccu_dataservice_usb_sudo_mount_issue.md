# USB mount issue **(sudo required)**

after inserting USB Sticks, the node `usb_detect` will need to mount the filesystem of USB-Stick.
This creates an issue regarding required root rights:

```bash
# requires to enter password INTERACTIVELY
sudo mount /dev/sda1 /targetdir

# udisksctl MIGHT BE executable for normal uses (without sudo authentication)
udisksctl /dev/sda1  # mounts at /media/<$USER>/<device-label>
```

This depends on **policy** configuration in file `org.freedesktop.UDisks2.policy` -
located at `/usr/share/polkit-1/actions`.
it can happen -
e.g. via remote `ssh` -
there are a different AUTHENTICATION mechanism active.  \
You can run into following Dialogs

- ==== AUTHENTICATING FOR org.freedesktop.udisks2.filesystem-mount-other-seat ===
  Authentication is required to mount Intenso Ultra Line (/dev/sda1)
  Authenticating as: Schumacher Georg (PT/PJ-TOP100) (sg82fe)
  Password:

or

- ==== AUTHENTICATING FOR org.freedesktop.udisks2.filesystem-mount ===
  Authentication is required to mount Intenso Ultra Line (/dev/sda1)
  Authenticating as: Schumacher Georg (PT/PJ-TOP100) (sg82fe)
  Password:

----

## Solutions

1. **worked BEST for me**:  \
   create file:  \
   `/etc/polkit-1/localauthority/50-local.d/90-allow-sudo-group-udisk-wo-password.pkla`

   with content:

   ```ini
   [udiskctlAllowRemoteUserMountDeviceswithoutPassword]
   #  Allow everyone:
   #  Identity=unix-user:*
   #
   #  Allow members of group sudo
   Identity=unix-group:sudo
   Action=org.freedesktop.udisks2.filesystem-mount;org.freedesktop.udisks2.filesystem-mount-system;org.freedesktop.udisks2.filesystem-mount-other-seat;org.freedesktop.UDisks2.filesystem-mount-fstab depending
   ResultAny=yes
   ResultInactive=yes
   ResultActive=yes
   ```

2. did not work:  \
   Add rules file as described e.g. here:
   <https://dynacont.net/documentation/linux/udisks2_polkit_Allow_unauthenticated_mounting/>

3. **worked**: BRUTAL WAY - **Not Recommended** - but works fine  \
   modify file  \
   `sudo nano /usr/share/polkit-1/actions/org.freedesktop.UDisks2.policy`

   search for

         <action id="org.freedesktop.udisks2.filesystem-mount">
         <action id="org.freedesktop.udisks2.filesystem-mount-system">
         <action id="org.freedesktop.udisks2.filesystem-mount-other-seat">

   change the restriction from AUTH_ADMIN to yes ... like here:

         <defaults>
           <allow_any>yes</allow_any>
           <allow_inactive>yes</allow_inactive>
           <allow_active>yes</allow_active>
         </defaults>

  There are hints to **not _!_ modify this file**,
  as it can be overwritten on software-updates.
  the ***correct*** way seems to create additional rule files or pkla files

- <https://askubuntu.com/questions/1129416/how-can-i-mount-partitions-in-thunar-without-password>

- background is the polkit, see
  - <https://www.freedesktop.org/software/polkit/docs/latest/polkit.8.html>
  - <https://wiki.archlinux.org/title/Polkit#Authorization_rules>
