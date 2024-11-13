# ArduTelex

ArduTelex is an minimum implementation of the iTelex protocol for a teletypewriter connected via a TW39 interface. It is possible to use the iTelex-Remote server to handle incomming calls (if no static IP address is available).

![20241105_180824](https://github.com/user-attachments/assets/2f0b3b79-2e49-452d-b057-a903b406de79)

The code is based on the following repositories:

https://github.com/soruh/centralexTW39

and

https://github.com/glsys/ArduinoTW39

--------------------------------------------------

Usage:

First you need to define if you want to build the Centralex (Remote Server) version or the normal version. The normal version is the version, that allows calling the ArduTelex via a public IP-Address or host-name on port 134. The Centralex version can be called via the iTelex-Centralex-Server (telexgateway.de or 78.47.222.94) on a telex-number specific port. If you want build the Centralex version add the code-line "#define CENTRALEX" to ArduTelex.h

In case of using the Centralex version you will need to put a file (owndata.txt) on the SD-Card into the folder /owndata. Full path: /owndata/owndata.txt
The file must contan the following content: 

Data:

123456

1234

+++


The first line must be "Data:"
The second line holds the Telex-Number of your Telex/TTY-Machine
The third line holds the server pin for your iTelex-Account
The forth line must contain "+++"

---------------------------------------------------------------------

It is also possible to define local address book. The address book files will be stored in the folder /pb. There will be one text-file per Telex-number. Each file name must be equal to the telex number (second line of the file) plus the file extension .txt

Example 123456.txt

ok

123456

Santa, North-Pole

2

192.168.178.90

134

\-

+++


Description:

1st line: ok

2nd line: tlex number

3rd line: your name and location

4th line: Peer-Type (1-Baudot Mode + Host-Name or 2-Baudot Mode + fixed IP-address)

5th line: IP-Address or Host-Name

6th line: Port number

7th line: \-

8th line: +++

--------------------------------------------------------------

You also need to assign the following Arduino Hardware Pins: Comutate pin (polarity of the line current), Recieve pin, and Send pin. In additon the levels (low, high) for the different states need to be defined. Those definition must be made in ArduTelex.h

If you are using a TW39-circut with a H-bridge as shown here:

![TW39-with-H-Bridge](https://github.com/user-attachments/assets/89c4e95f-48fa-47c2-8236-cd1819ea3e6b)

Then you need use this definition "#define SOLID_STATE_TW39", if you use a "traditional" TW39 circuitry then you comment out the SOLID_STATE_TW39 definition. The actual assignment of the low/high levels is done in ArduTelex.h from this code line " #ifdef SOLID_STATE_TW39       // solid state TW39 circuitry" on.


More descriptions will follow ...




