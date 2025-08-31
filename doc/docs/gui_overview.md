# VNC vs Xpra

We offer two different options for displaying the graphical user interface: xpra or VNC. Both are different and have their specific use cases.

* The VNC option enables viewing an entire Ubuntu Desktop within your browser. Useful in case you are running this project on a local machine. Refer [VNC setup](howToVNC.md) for details and configuration.
* The xpra option enables forwarding single X11 windows from your terminal. Its a bit more lightweight and useful if you are running this project on a remote machine. It requires your user to be ported into the docker container, however. This might not work on all configurations. Refer [Xpra setup](howToXpra.md) for details and configuration.

