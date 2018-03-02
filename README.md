This project is clone of another project: [Arduino Vna](https://hamprojects.wordpress.com/2016/02/21/hf-arduino-vna-english-version ) which is a clone of another project: [ra4nal.qrz.ru](http://ra4nal.qrz.ru/vna.shtml) :-)
Hardware is based on the original project. There is a little modification of the [Low Pass Filter](https://www.google.pl/url?sa=t&rct=j&q=&esrc=s&source=web&cd=7&ved=0ahUKEwjrhpG5v87ZAhVDY1AKHbH4DB8QFghCMAY&url=https%3A%2F%2Fwww.edn.com%2FPdf%2FViewPdf%3FcontentItemId%3D4441389&usg=AOvVaw0NhQ0TK24KmhZ63Z3tyB0r) 

Spice notation:
 
R1 1 0 200  
C1 1 0 15pF  
L1 1 2 0.82uH  
C2 2 3 20pF  
L2 3 0 270nH  
L3 2 4 1uH  
C3 4 0 33pF  
L4 4 5 0.82uH  
C4 4 0 62pF  
RL 4 0 50

![](https://www.edn.com/ContentEETimes/Images/EDN/Design%20How-To/AD9851/Chebyshev%20low%20pass%20filter%2065%20MHz%20trap.jpg)

## Bluetooth
Additional step is needed to change default baud rate from 9600 to 115200.  
Connect the bluetooth module to PC via some USB/TTL uart converter.  
Check if transmission works by typing `AT` command.   
The module should response `OK`.  
Copy paste `"AT+BAUDX8"` to terminal this will set transmission to 115200bps.  
Set also name of the module: `AT+NAMEmodulename`

