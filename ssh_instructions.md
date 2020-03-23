# Generate ssh key-pair

`ssh-keygen`

Replace rover6_dul_id_rsa with the rover name

```(bash)
Generating public/private rsa key pair.
Enter file in which to save the key (/home/pi/.ssh/id_rsa): /home/pi/.ssh/rover6_dul_id_rsa
Created directory '/home/pi/.ssh'.
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in /home/pi/.ssh/rover6_dul_id_rsa.
Your public key has been saved in /home/pi/.ssh/rover6_dul_id_rsa.pub.
The key fingerprint is:
SHA256:-----------------------/------------------- pi@dul
The key's randomart image is:
```

# Setup SSH key directories
Copy `/home/pi/.ssh/rover6_dul_id_rsa.pub` to `/home/pi/.ssh/authorized_keys`

Copy `rover6_dul_id_rsa` and `rover6_dul_id_rsa.pub` to your local machine's `~/.ssh` directory.

Test the log in: `ssh -i ~/.ssh/rover6_dul_id_rsa pi@dul.local`

# Disable password login
`sudo nano /etc/ssh/sshd_config`

Search for `#PasswordAuthentication yes`

Change to `PasswordAuthentication no`

Save and restart the ssh service:

`sudo service ssh restart`
