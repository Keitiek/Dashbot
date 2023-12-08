### Remote Login, or Remote Access

To login into Jetson, find out what Jetson's username, password and local IP address is.
Then, use this command:

```ssh
ssh dashbot@10.0.3.242
```

If you want to work with Arduino, you'll need to allow external display. In this case, use this command instead:

```ssh
ssh -X dashbot@10.0.3.242
```
